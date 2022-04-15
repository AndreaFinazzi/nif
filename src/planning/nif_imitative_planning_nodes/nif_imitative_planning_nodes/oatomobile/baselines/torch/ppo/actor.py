from typing import Mapping
import torch
from oatomobile.torch.networks.mlp import MLP
from oatomobile.torch.networks.perception import MobileNetV2
from oatomobile.torch import transforms

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from a2c_ppo_acktr.distributions import Bernoulli, Categorical, DiagGaussian
from a2c_ppo_acktr.utils import init


class Flatten(nn.Module):
    def forward(self, x):
        return x.view(x.size(0), -1)


class Policy(nn.Module):
    def __init__(self, input_shape, num_outputs, base=None, base_kwargs=None):
        super(Policy, self).__init__()
        base = CNNBase
        self.base = base(input_shape, num_outputs, **base_kwargs)
        #don't need action space since it's always box      
        self.dist = DiagGaussian(self.base.output_size, num_outputs)
       
    @property
    def is_recurrent(self):
        return self.base.is_recurrent

    @property
    def recurrent_hidden_state_size(self):
        """Size of rnn_hx."""
        return self.base.recurrent_hidden_state_size

    def forward(self, inputs, rnn_hxs, masks):
        raise NotImplementedError

    def act(self, inputs, rnn_hxs, masks, deterministic=False):
        value, actor_features, rnn_hxs = self.base(inputs, rnn_hxs, masks)
        dist = self.dist(actor_features)

        if deterministic:
            action = dist.mode()
        else:
            action = dist.sample()

        #add ReLU
        # action = nn.ReLU()(action)

        action_log_probs = dist.log_probs(action)
        dist_entropy = dist.entropy().mean()

        return value, action, action_log_probs, rnn_hxs

    def get_value(self, inputs, rnn_hxs, masks):
        value, _, _ = self.base(inputs, rnn_hxs, masks)
        return value

    def evaluate_actions(self, inputs, rnn_hxs, masks, action):
        value, actor_features, rnn_hxs = self.base(inputs, rnn_hxs, masks)
        dist = self.dist(actor_features)

        action_log_probs = dist.log_probs(action)
        dist_entropy = dist.entropy().mean()

        return value, action_log_probs, dist_entropy, rnn_hxs


class NNBase(nn.Module):
    def __init__(self, recurrent, recurrent_input_size, hidden_size):
        super(NNBase, self).__init__()

        self._hidden_size = hidden_size
        self._recurrent = recurrent

        if recurrent:
            self.gru = nn.GRU(recurrent_input_size, hidden_size)
            for name, param in self.gru.named_parameters():
                if 'bias' in name:
                    nn.init.constant_(param, 0)
                elif 'weight' in name:
                    nn.init.orthogonal_(param)

    @property
    def is_recurrent(self):
        return self._recurrent

    @property
    def recurrent_hidden_state_size(self):
        if self._recurrent:
            return self._hidden_size
        return 1

    @property
    def output_size(self):
        return self._hidden_size

    def _forward_gru(self, x, hxs, masks):
        if x.size(0) == hxs.size(0):
            x, hxs = self.gru(x.unsqueeze(0), (hxs * masks).unsqueeze(0))
            x = x.squeeze(0)
            hxs = hxs.squeeze(0)
        else:
            # x is a (T, N, -1) tensor that has been flatten to (T * N, -1)
            N = hxs.size(0)
            T = int(x.size(0) / N)

            # unflatten
            x = x.view(T, N, x.size(1))

            # Same deal with masks
            masks = masks.view(T, N)

            # Let's figure out which steps in the sequence have a zero for any agent
            # We will always assume t=0 has a zero in it as that makes the logic cleaner
            has_zeros = ((masks[1:] == 0.0) \
                            .any(dim=-1)
                            .nonzero()
                            .squeeze()
                            .cpu())

            # +1 to correct the masks[1:]
            if has_zeros.dim() == 0:
                # Deal with scalar
                has_zeros = [has_zeros.item() + 1]
            else:
                has_zeros = (has_zeros + 1).numpy().tolist()

            # add t=0 and t=T to the list
            has_zeros = [0] + has_zeros + [T]

            hxs = hxs.unsqueeze(0)
            outputs = []
            for i in range(len(has_zeros) - 1):
                # We can now process steps that don't have any zeros in masks together!
                # This is much faster
                start_idx = has_zeros[i]
                end_idx = has_zeros[i + 1]

                rnn_scores, hxs = self.gru(
                    x[start_idx:end_idx],
                    hxs * masks[start_idx].view(1, -1, 1))

                outputs.append(rnn_scores)

            # assert len(outputs) == T
            # x is a (T, N, -1) tensor
            x = torch.cat(outputs, dim=0)
            # flatten
            x = x.view(T * N, -1)
            hxs = hxs.squeeze(0)

        return x, hxs

class CNNBase(NNBase):
    def __init__(self, input_shape, num_outputs, recurrent=False, hidden_size=512, include_depth=True):
        
        super(CNNBase, self).__init__(recurrent, hidden_size, hidden_size)
        # super(CNNBase, self).__init__(recurrent, num_outputs, num_outputs)

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        init_ = lambda m: init(m, nn.init.orthogonal_, lambda x: nn.init.
                        constant_(x, 0), nn.init.calculate_gain('relu'))
        
        # self.output_shape = output_shape #should be 2, velocity and steer
        self.include_depth = include_depth

        if self.include_depth:
            self._encoder = MobileNetV2(num_classes=128, in_channels=4)
        else:
            self._encoder = MobileNetV2(num_classes=128, in_channels=3)
        self._encoder.to(device)
        # print('next(self._encoder.parameters()).is_cuda', next(self._encoder.parameters()).is_cuda)
        # import pdb; pdb.set_trace()

        self._merger = MLP(
        input_size = input_shape[1]*input_shape[0] + 128,
        # output_sizes=[64, 64, num_outputs],
        output_sizes=[512, 512, 512, 512, hidden_size],
        activation_fn=nn.ReLU,
        dropout_rate=None,
        activate_final=True,
        )
        self._merger.to(device)

        init_ = lambda m: init(m, nn.init.orthogonal_, lambda x: nn.init.
                        constant_(x, 0))

        self.critic_linear = nn.Sequential(
                                            init_(nn.Linear(hidden_size, 256)),
                                            init_(nn.Linear(256, 128)),
                                            init_(nn.Linear(128, 64)), 
                                            init_(nn.Linear(64, 32)), 
                                            init_(nn.Linear(32, 16)), 
                                            init_(nn.Linear(16, 8)), 
                                            init_(nn.Linear(8, 4)),
                                            init_(nn.Linear(4, 1))
        ) 
        # self.critic_linear = init_(nn.Linear(hidden_size, 1))

        self.critic_linear.to(device)
        self.train()

    def forward(self, x, rnn_hxs, masks):
    #context is input
    # def forward(self, **context: torch.Tensor) -> torch.Tensor:
        """Returns the contextual parameters of the conditional density estimator.
        Args:
        visual_features: The visual input, with shape `[B, H, W, 5]`.
        Returns:
        The contextual parameters of the conditional density estimator.
        """
        #need to convert context from tensor to dictionary
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        context = {}

        # print('x in forward', x.size())
        print('x', x.shape)

        context["image_front_color"] = x[:, 0, :, :, :].to(device)
        context["image_front_depth"] = x[:, 1, 0, :, :].to(device)
        # print('x[2, :, :10, 0]', x[:, 2, :, :10, 0].size())
        # context["player_past"] = x[:, 2, :, :10, 0].permute(0, 2, 1).to(device)
        # print('x[:, 2, 0, :7, :10]', x[:, 2, 0, :7, :10].shape)
        # print('x', x.shape)
        context["player_past"] = x[:, 2, 0, :7, :10].permute(0, 2, 1).to(device)

        # print('context["image_front_color"]', context["image_front_color"].size())
        # print('context["image_front_depth"]', context["image_front_depth"].size())
        # print('context["player_past"]', context["player_past"].size())
        # import pdb; pdb.set_trace()

        context = self.transform(context) #transform in

        # Parses context variables.
        if not "visual_features" in context:
          raise ValueError("Missing `visual_features` keyword argument.")
        #if not "player_past" in context:
        #  raise ValueError("Missing `player_past` keyword argument.")
        visual_features = context.get("visual_features")
        player_past = context.get("player_past")
        # print('player_past shape', player_past.shape)
        # print('context["player_past"]', context["player_past"])
        # import pdb; pdb.set_trace()
        # Encodes the visual input.
        visual_features = self._encoder(visual_features)
        # print('visual_features', visual_features)
        # print('self._encoder', self._encoder)
        # import pdb; pdb.set_trace()

        # Merges visual input logits and vector inputs.
        # print(visual_features.shape)
        player_past = torch.reshape(player_past, (player_past.shape[0], player_past.shape[1]*player_past.shape[2]))
        # print(player_past.shape)
        visual_features = torch.cat(  # pylint: disable=no-member
            tensors=[
                visual_features,
                player_past
            ],
            dim=-1
        )

        # The decoders initial state.
        x = self._merger(visual_features) #action is cmd_vel
        # print('x after merger', x.shape)
        # x = nn.Sigmoid()(x)

        if self.is_recurrent:
          x, rnn_hxs = self._forward_gru(x, rnn_hxs, masks)

        return self.critic_linear(x), x, rnn_hxs
     
    def transform(self, sample):
        """Prepares variables for the interface of the model.
        Args:
          sample: (keyword arguments) The raw sample variables.
        Returns:
          The processed sample.
        """

        # Renames `lidar` to `visual_features`.
        if self.include_depth:
          sample["visual_features"] = torch.cat((sample.pop("image_front_color"),
                                                torch.unsqueeze(sample.pop("image_front_depth"), dim=1)), dim=1)
            # sample["visual_features"] = torch.cat((sample.pop("image_front_color"),
            #                        sample.pop("image_front_depth")), dim=1)
        else:
          sample["visual_features"] = sample.pop("image_front_color")

        
        # Preprocesses the visual features.
        if "visual_features" in sample:
          sample["visual_features"] = transforms.transpose_visual_features(
              transforms.downsample_visual_features(
                  visual_features=sample["visual_features"],
                  output_shape=(100, 100),
              ))

        return sample