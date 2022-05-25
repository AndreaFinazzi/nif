from typing import Mapping
import torch
import torch.nn as nn
from oatomobile.torch.networks.mlp import MLP
from oatomobile.torch.networks.perception import MobileNetV2
from oatomobile.torch import transforms


class BCModel(nn.Module):

    def __init__(self, include_depth, output_shape, goal_directed):

        super(BCModel, self).__init__()

        self.output_shape = output_shape
        self.include_depth = include_depth
        self.goal_directed = goal_directed

        if self.include_depth:
            self._encoder = MobileNetV2(num_classes=128, in_channels=4)
        else:
            self._encoder = MobileNetV2(num_classes=128, in_channels=3)

        additional_pt = output_shape[1] if goal_directed else 0
        
        self._merger = MLP(
            input_size = output_shape[1]*output_shape[0] + 128 + additional_pt,
            output_sizes=[400, 300, 200, 100, output_shape[0]*output_shape[1]],
            activation_fn=nn.ReLU,
            dropout_rate=None,
            activate_final=False,
        )

    def forward(self, **context: torch.Tensor) -> torch.Tensor:
        """Returns the contextual parameters of the conditional density estimator.
        Args:
        visual_features: The visual input, with shape `[B, H, W, 5]`.
        Returns:
        The contextual parameters of the conditional density estimator.
        """

        # Parses context variables.
        if not "visual_features" in context:
          raise ValueError("Missing `visual_features` keyword argument.")
        #if not "player_past" in context:
        #  raise ValueError("Missing `player_past` keyword argument.")
        visual_features = context.get("visual_features")
        player_past = context.get("player_past")
        # Encodes the visual input.
        visual_features = self._encoder(visual_features)
        # Merges visual input logits and vector inputs.
        player_past = torch.reshape(player_past, (player_past.shape[0], player_past.shape[1]*player_past.shape[2]))
        if self.goal_directed:
            future_truth = context.get("player_future")
            future = torch.reshape(future_truth[:, -1, :], (player_past.shape[0], self.output_shape[-1]))
            visual_features = torch.cat(  # pylint: disable=no-member
                tensors=[
                    visual_features,
                    player_past,
                    future
                ],
                dim=-1
            )
        else:
            visual_features = torch.cat(  # pylint: disable=no-member
                tensors=[
                    visual_features,
                    player_past
                ],
                dim=-1
            )

        # The decoders initial state.
        trajectories = self._merger(visual_features)
        return trajectories.view(-1, self.output_shape[0], self.output_shape[1])

    def transform(self, sample):
        """Prepares variables for the interface of the model.
        Args:
          sample: (keyword arguments) The raw sample variables.
        Returns:
          The processed sample.
        """

        # Renames `lidar` to `visual_features`.
        if self.include_depth:
          # print(sample["image_front_color"].shape , sample["image_front_depth"].shape )
          if sample["image_front_depth"].shape[1] == 3:
              sample["image_front_depth"] = sample["image_front_depth"][:, 0]
          sample["visual_features"] = torch.cat((sample.pop("image_front_color"),
              torch.unsqueeze(sample.pop("image_front_depth"), dim=1)), dim=1)
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

