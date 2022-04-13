# Copyright 2020 The OATomobile Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Defines a discriminative model for the conditional imitation learner."""

from typing import Mapping
from typing import Optional
from typing import Sequence
from typing import Tuple
import numpy as np
from typing import Union

import torch
import torch.distributions as D
import torch.nn as nn
import matplotlib.path as mpltPath
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable
from oatomobile.torch import transforms
from oatomobile.torch import types
from oatomobile.torch.networks.mlp import MLP
from oatomobile.torch.networks.perception import MobileNetV2
from oatomobile.torch.networks.sequence import AutoregressiveFlow


class ImitativeModel(nn.Module):
    """A `PyTorch` implementation of an imitative model."""

    def __init__(
        self,
        include_depth=True,
        num_pos_dim=3,
        input_shape=[10, 3],
        output_shape: types.Shape = (4, 2),
	include_lidar=False,
        include_rgb=True
    ) -> None:
        """Constructs a simple imitative model.

        Args:
          output_shape: The shape of the base and
            data distribution (a.k.a. event_shape).
        """
        super(ImitativeModel, self).__init__()
        self._output_shape = output_shape
        self._include_depth = include_depth
        self._num_pos_dim = num_pos_dim
        self._include_lidar = include_lidar
        self._include_rgb = include_rgb
        # FACTOR WITH ABSL FLAGS
        self.input_shape = input_shape
        lidar_inp =  0
        visual_inp = 0
        assert self._include_rgb or self._include_depth or self._include_lidar, "Input must include at least RGB, depth, or LiDAR"
        
        # The convolutional encoder model.
	# increased num classes to 256
        if self._include_depth and self._include_rgb:
            visual_inp = 128
            self._encoder = MobileNetV2(num_classes=visual_inp, in_channels=4)
        elif self._include_rgb:
            visual_inp = 128
            self._encoder = MobileNetV2(num_classes=visual_inp, in_channels=3)
        elif self._include_depth:
            visual_inp = 128
            self._encoder = MobileNetV2(num_classes=visual_inp, in_channels=1)
        
        print("INPUT EXPECTED TO BE SHAPE", input_shape)
        
        # Merges the encoded features and the vector inputs.
        if include_lidar:
            print('creating a lidar encoder')
            lidar_inp = 64
            self._lidar_encoder = MobileNetV2(num_classes=64, in_channels=3)
	# added dropout, removed final activation, increased dim(z) from 64
        
        self._merger = MLP(
            # input_size=128, Add the output_shape product
            input_size= lidar_inp + self._num_pos_dim*self.input_shape[0] + visual_inp,
            output_sizes=[64, 64, 64],
            activation_fn=nn.ReLU,
            dropout_rate=None,
            activate_final=True,
        )

        # The decoder recurrent network used for the sequence generation.
        self._decoder = AutoregressiveFlow(
            output_shape=self._output_shape,
            hidden_size=64,
        )

    def to(self, *args, **kwargs):
        """Handles non-parameter tensors when moved to a new device."""
        self = super().to(*args, **kwargs)
        self._decoder = self._decoder.to(*args, **kwargs)
        return self

    def forward(
            self,
            num_steps: int,
            goal: Optional[torch.Tensor] = None,
            lr: float = 1e-1,
            epsilon: float = 1.0,
            return_scores=False,
            **context: torch.Tensor) -> Union[torch.Tensor, Sequence[torch.Tensor]]:
        """Returns a local mode from the posterior.

        Args:
          num_steps: The number of gradient-descent steps for finding the mode.
          goal: The locations of the the goals.
          epsilon: The tolerance parameter for the goal.
          context: (keyword arguments) The conditioning
            variables used for the conditional flow.

        Returns:
          A mode from the posterior, with shape `[D, 2]`.
        """
        batch_size = context["player_future"].shape[0]

        # Sets initial sample to base distribution's mean.
        x = self._decoder._base_dist.sample().clone().detach().repeat(
            batch_size, 1).view(
                batch_size,
                *self._output_shape,
        )
        x.requires_grad = True
        # The contextual parameters, caches for efficiency.
        z = self._params(**context)

        # Initialises a gradient-based optimiser.
        optimizer = optim.Adam(params=[x], lr=lr)

        # Stores the best values.
        x_best = x.clone()
        loss_best = torch.ones(()).to(x.device) * 1000.0  # pylint: disable=no-member

        for _ in range(num_steps):
            # Resets optimizer's gradients.
            optimizer.zero_grad()
            # Operate on `y`-space.
            y, _ = self._decoder._forward(x=x, z=z)
            # Calculates imitation prior.
            _, log_prob, logabsdet = self._decoder._inverse(y=y, z=z)
            imitation_prior = torch.mean(log_prob - logabsdet)  # pylint: disable=no-member q(x|phi)
            # Calculates goal likelihodd.
            goal_likelihood = 0.0
            if goal is not None:
                goal_likelihood = self._goal_likelihood(y=y, goal=goal, epsilon=epsilon)
                assert imitation_prior.shape == goal_likelihood.shape
            loss = -(imitation_prior + goal_likelihood)
            # Backward pass.
            loss.backward(retain_graph=True)
            # Performs a gradient descent step.
            optimizer.step()
            # Book-keeping
            if loss < loss_best:
                x_best = x.clone()
                loss_best = loss.clone()
        y, _ = self._decoder._forward(x=x_best, z=z)
        if return_scores:
            return y, loss_best
        return y

    def _goal_likelihood(self,
                         y: torch.Tensor,
                         goal: torch.Tensor,
                         comp=0,
                         **hyperparams) -> torch.Tensor:
        """Returns the goal-likelihood of a plan `y`, given `goal`.

        Args:
          y: A plan under evaluation, with shape `[B, T, 2]`.
          goal: The goal locations, with shape `[B, K, 2]`.
          hyperparams: (keyword arguments) The goal-likelihood hyperparameters.

        Returns:
          The log-likelihood of the plan `y` under the `goal` distribution.
        """
        # Parses tensor dimensions.
        B, K, _ = goal.shape

        # Fetches goal-likelihood hyperparameters.
        epsilon = hyperparams.get("epsilon", 1.0)

        # TODO(filangel): implement other goal likelihoods from the DIM paper
        # Initializes the goal distribution.
        goal_distribution = D.MixtureSameFamily(
            mixture_distribution=D.Categorical(
                probs=torch.ones((B, K)).to(goal.device)),  # pylint: disable=no-member
            component_distribution=D.Independent(
                D.Normal(loc=goal, scale=torch.ones_like(goal) * epsilon),  # pylint: disable=no-member
                reinterpreted_batch_ndims=1,
            ))
        if comp:
            return goal_distribution.log_prob(y[:, -1, :])
        return torch.mean(goal_distribution.log_prob(y[:, -1, :]), dim=0)  # pylint: disable=no-member

    def _region_based_planning(self, num_steps: int, goal_polygon: Optional[torch.Tensor] = None,
                               lr: float = 1e-1, epsilon: float = 1.0, return_scores=False,
                               **context: torch.Tensor) -> torch.Tensor:
        """Returns a local mode from the posterior with a region based system.

        Args:
          num_steps: The number of gradient-descent steps for finding the mode.
          goal: The locations of the the goals.
          epsilon: The tolerance parameter for the goal.
          context: (keyword arguments) The conditioning
            variables used for the conditional flow.

        Returns:
          A mode from the posterior, with shape `[D, 2]`.
        """
        if not "visual_features" in context:
            raise ValueError("Missing `visual_features` keyword argument.")

        batch_size = context["visual_features"].shape[0]

        # Sets initial sample to base distribution's mean.
        x = self._decoder._base_dist.sample(torch.Size([batch_size])).clone().detach().view(
            batch_size, *self._output_shape)

        # MAKE POLYGON EDGES
        goal_polygon = goal_polygon.repeat((batch_size, 1, 1))
        goal_polygon_closed = torch.cat([goal_polygon, goal_polygon[:, [0]]], dim=1)
        end = goal_polygon_closed[..., 1:, :]
        start = goal_polygon_closed[..., :-1, :]
        segment_vector = end - start
        x.requires_grad = True

        # The contextual parameters, caches for efficiency.
        z = self._params(**context)

        # Initialises a gradient-based optimiser.
        optimizer = optim.Adam(params=[x], lr=lr)

        # Stores the best values.
        x_best = x.clone()
        loss_best = torch.ones((batch_size)).to(x.device) * 1000.0  # pylint: disable=no-member

        for _ in range(num_steps):
            # Resets optimizer's gradients.
            optimizer.zero_grad()
            # Operate on `y`-space.
            # y, _, mu_t, sig_t = self._decoder._forward(x=torch.stack(p), z=z, return_rollouts=True)
            y, _, mu_t, sig_t = self._decoder._forward(x=x, z=z, return_rollouts=True)

            inclusion_mask = self.mpl_in_polygon(mu_t[-1][:, :2].cpu().detach().numpy(),
                                                 goal_polygon[0].cpu().detach().numpy(), flip=True)

            # Calculates imitation prior.
            _, log_prob, logabsdet = self._decoder._inverse(y=y, z=z)
            imitation_prior = log_prob - logabsdet  # pylint: disable=no-member q(x|phi)

            # Calculates goal likelihodd.
            goal_likelihood = torch.zeros((batch_size)).to(x.device)
            if goal_polygon is not None and not inclusion_mask.all():

                goal_likelihood = self._region_goal(inclusion_mask=torch.BoolTensor(inclusion_mask).to(x.device),
                                                    y=y[:, -1], segment=segment_vector, mu_t=mu_t[-1], sig_t=sig_t[-1],
                                                    start=start, comp=1)
                assert imitation_prior.shape == goal_likelihood.shape

            loss = -(imitation_prior + goal_likelihood)
            # Backward pass.
            loss.mean().backward(retain_graph=True)
            # Performs a gradient descent step.
            optimizer.step()
            # Book-keeping
            x_best = torch.where(loss.view(-1, 1, 1) < loss_best.view(-1, 1, 1), x, x_best).clone()
            loss_best = torch.where(loss < loss_best, loss, loss_best).clone()

        y, _ = self._decoder._forward(x=x_best, z=z)
        if return_scores:
            return y, loss_best
        return y

    def _region_goal(self, inclusion_mask, segment: torch.Tensor, mu_t, sig_t, start, y,
                     comp=0, **hyperparams) -> torch.Tensor:
        """Returns the goal-likelihood of a plan `y`, given `goal`.

            Args:
              y: A plan under evaluation, with shape `[B, T, 2]`.
              goal_polygon: The vertices of the region to shape the goal with `[B, K, 2]`.
              hyperparams: (keyword arguments) The goal-likelihood hyperparameters.

            Returns:
              The log-likelihood of the plan `y` under the `goal` distribution.
        """
        mu_T = mu_t[:, :2]
        sig_T = sig_t[:, :2]

        cov_mat = torch.diag_embed(sig_T)
        mu_T_tiled = mu_T.unsqueeze(dim=1).repeat((1, segment.shape[1], 1))
        cov_mat_tiled = cov_mat.unsqueeze(dim=1).repeat((1, segment.shape[1], 1, 1))

        q_T = D.MultivariateNormal(loc=mu_T_tiled, covariance_matrix=cov_mat_tiled)
        log_qx = torch.max(q_T.log_prob(y[:, :2].unsqueeze(dim=1).repeat((1, segment.shape[1], 1))), dim=1)[0]
        prec_mat = torch.inverse(cov_mat)

        unum = -torch.einsum('bmd,bde,bme->bm', segment, prec_mat, start - mu_T[:, None, :])
        udenom = torch.einsum('bmd,bde,bme->bm', segment, prec_mat, segment)
        u_edges = self.safe_divide(unum, udenom)
        u_edges = torch.clamp(u_edges, 0, 1)
        xy_edges = start + torch.einsum('bm,bmd->bmd', u_edges, segment)
        log_q_T_outside = torch.max(q_T.log_prob(xy_edges), dim=1)[0]
        log_q_T_inside = torch.max(q_T.log_prob(mu_T_tiled), dim=1)[0]
        log_prob_As = torch.where(inclusion_mask, log_q_T_inside, log_q_T_outside)
        log_prob = log_prob_As - log_qx
        if comp:
            return log_prob
        return log_prob.mean()

    def safe_divide(self, numerator, denominator):
        x = denominator
        x_ok = ~x.eq(torch.zeros(x.shape)
                     .to(x.device))

        def f(x): return numerator / x
        safe_f = torch.zeros_like
        safe_x = torch.where(x_ok, x, torch.ones_like(x))
        u_safe = torch.where(x_ok, f(safe_x), safe_f(x))
        return u_safe

    def mpl_in_polygon(self, points, polygon, flip=False):
        path = mpltPath.Path(polygon)
        if flip:
            points = np.flip(points, axis=1)
        inside2 = path.contains_points(points)
        return inside2

    def _params(self, **context: torch.Tensor) -> torch.Tensor:
        """Returns the contextual parameters of the conditional density estimator.

        Args:
          visual_features: The visual input, with shape `[B, H, W, 5]`.

        Returns:
          The contextual parameters of the conditional density estimator.
        """

        # Parses context variables.
        include_visual = self._include_rgb or self._include_depth
        player_past = context.get("player_past")
        if self._include_lidar:
            lidar_feats = self._lidar_encoder(context.get("lidar_features"))
        if include_visual:
            visual_features = self._encoder(context.get("visual_features"))
        
        # Merges visual input logits and vector inputs.
        player_past = torch.reshape(player_past, (player_past.shape[0], player_past.shape[1]*player_past.shape[2]))
        
        if include_visual and self._include_lidar:
            visual_features = torch.cat(  # pylint: disable=no-member
                tensors=[
                    visual_features,
                    player_past,
                    lidar_feats
                ],
                dim=-1
            )
        elif include_visual:
            visual_features = torch.cat(  # pylint: disable=no-member
                tensors=[
                    visual_features,
                    player_past
                ],
                dim=-1
            )
        else:
            visual_features = torch.cat(  # pylint: disable=no-member
                tensors=[
                    player_past,
                    lidar_feats
                ],
                dim=-1
            )

        # The decoders initial state.
        visual_features = self._merger(visual_features)

        return visual_features

    def transform(
        self,
        sample: Mapping[str, types.Array],
    ) -> Mapping[str, torch.Tensor]:
        """Prepares variables for the interface of the model.

        Args:
          sample: (keyword arguments) The raw sample variables.

        Returns:
          The processed sample.
        """
        # Preprocesses the target variables.
        if "player_future" in sample:
            sample["player_future"] = transforms.downsample_target(
                player_future=sample["player_future"],
                num_timesteps_to_keep=self._output_shape[-2],
            )
        if "image_front_color" in sample:
            if sample["image_front_color"].shape[-1] == 3:
                sample["image_front_color"] = sample["image_front_color"].permute(0, 3, 1, 2)
        if "image_front_depth" in sample:
            if sample["image_front_depth"].shape[-1] == 3 or sample["image_front_depth"].shape[-1] == 1:
                sample["image_front_depth"] = sample["image_front_depth"].permute(0, 3, 1, 2)


        if self._include_depth and self._include_rgb:
            # Assuming 3 channel depth is repeated arbitrarily
            if sample["image_front_depth"].shape[1] == 3:
                sample["image_front_depth"] = sample["image_front_depth"][:, 0].unsqueeze(1)
            
            sample["visual_features"] = torch.cat((sample.pop("image_front_color"),
                                                  (sample.pop("image_front_depth"))), dim=1)
        elif self._include_rgb:
            sample["visual_features"] = sample.pop("image_front_color")
        elif self._include_depth:
            if sample["image_front_depth"].shape[1] == 3:
                sample["image_front_depth"] = sample["image_front_depth"][:, 0].unsqueeze(1)
            sample["visual_features"] = sample.pop("image_front_depth")

        if "overhead_features" in sample:
            sample["lidar_features"] = sample.pop("overhead_features")

        # Preprocesses the visual features.
        
        """
        if "visual_features" in sample:
            sample["visual_features"] = transforms.transpose_visual_features(
                transforms.downsample_visual_features(
                    visual_features=sample["visual_features"],
                    output_shape=(200, 200),
                ))
        """
        # removing arbitrary transpose: dashora7 
        if "visual_features" in sample:
            sample["visual_features"] = transforms.downsample_visual_features(
                    visual_features=sample["visual_features"],
                    output_shape=(100, 100),
            )
        
        if "lidar_features" in sample:
            sample["lidar_features"] = transforms.downsample_visual_features(
                visual_features=sample["lidar_features"],
                output_shape=(100, 100),
            )
        return sample

    def trajectory_library_plan(
            self,
            centroids,
            context,
            goal=None,
            epsilon=1,
            region_based=False,
            rho=None,
            phi=1,
            costmap=None,
            costmap_only=False,
            triage=None,
            past_goal=None,
            chi=1,
            stdev=1):
        """centroids: a batch of learned centroids from the trajectory library
           context: the context vector used for inverse prob calculation

           returns the centroid from the learned clusters that maximizes the inverse prob
        """
        # print('centroids.shape', centroids.shape)
        # print('context.shape', context.shape)
        # import pdb; pdb.set_trace()
        _, log_prob, logabsdet, mu_t, sig_t = self._decoder._inverse(
            y=centroids,
            z=context,
            return_rollouts=True
        )
        imitation_prior = log_prob - logabsdet
        # Calculates goal likelihood.
        goal_likelihood = 0
        costmap_values = 0
        gaussian_encouragement_score = 0
        if past_goal is not None:
            gaussian_encouragement_score = chi * self._goal_likelihood(
                y=centroids,
                goal=past_goal,
                comp=1,
                epsilon=stdev
            )
        if costmap is not None:
            costmap_values = phi * costmap
        if region_based:
            # print(goal.shape)
            goal_polygon = goal.repeat((centroids.shape[0], 1, 1))
            inclusion_mask = self.mpl_in_polygon(
                mu_t[-1][:, :2].cpu().detach().numpy(),
                goal_polygon[0].cpu().detach().numpy(),
                flip=True
            )
            goal_polygon_closed = torch.cat([goal_polygon, goal_polygon[:, [0]]], dim=1)
            end = goal_polygon_closed[..., 1:, :]
            start = goal_polygon_closed[..., :-1, :]
            segment_vector = end - start
            # print(segment_vector.device, mu_t[-1].device, centroids.device, sig_t[-1].device)
            goal_likelihood = self._region_goal(
                inclusion_mask=torch.BoolTensor(inclusion_mask).to(mu_t[-1].device),
                y=centroids[:, -1],
                segment=segment_vector,
                mu_t=mu_t[-1],
                sig_t=sig_t[-1],
                start=start,
                comp=1)
        else:
            if goal is not None:
                goal_likelihood = self._goal_likelihood(y=centroids, goal=goal, comp=1, epsilon=epsilon)
                assert imitation_prior.shape == goal_likelihood.shape

        if rho:
            speeds = self.speed_heuristic(centroids)
            loss = (imitation_prior + epsilon*goal_likelihood + rho*speeds) - costmap_values + gaussian_encouragement_score
        else:
            loss = (imitation_prior + epsilon*goal_likelihood) - costmap_values + gaussian_encouragement_score

        if costmap_only:
            loss -= imitation_prior
        elif triage is not None:
            topk_val = torch.mean(torch.topk(costmap_values, k=25, dim=0)[0]).item()
            print('topk_val', topk_val)
            if topk_val > triage:
                loss -= imitation_prior
                # print('*' * 100)
                # print('*' * 100)
                # print('use costmap only')
                # print('*' * 100)
                # print('*' * 100)
            else:
                loss += costmap_values
                # print('*' * 100)
                # print('*' * 100)
                # print('use imitation prior only')
                # print('*' * 100)
                # print('*' * 100)

        # if type(costmap_values) is list:
        #   print('costmap_values:', costmap_values[:20])

        # print('imitation_priors:', imitation_prior[:20])
        # if goal is not None:
        #    print('goal_likelihood:', goal_likelihood[:20])

        prob_idx = torch.argmax(loss, dim=0)
        # print(loss[prob_idx])
        return centroids[prob_idx], loss

    def speed_heuristic(self, trajectories):
        d = trajectories.device
        trajectories = trajectories.clone().detach().cpu().numpy()
        trajectory_diff = np.square(np.diff(trajectories, axis=1))
        trajectory_dist = np.sum(trajectory_diff, axis=(1, 2))
        return torch.FloatTensor(trajectory_dist).to(d)

    def mse_eval(self, prediction, ground_truth):
        return torch.norm(prediction - ground_truth) / prediction.shape[0]

    def multiseed_forward(
            self,
            num_steps: int,
            goal: Optional[torch.Tensor] = None,
            lr: float = 1e-1,
            epsilon: float = 1.0,
            **context: torch.Tensor) -> Union[torch.Tensor, Sequence[torch.Tensor]]:
        """Returns a local mode from the posterior by instantiated multiple seeds

        Args:
          num_steps: The number of gradient-descent steps for finding the mode.
          goal: The locations of the the goals.
          epsilon: The tolerance parameter for the goal.
          context: (keyword arguments) The conditioning
            variables used for the conditional flow.

        Returns:
          A mode from the posterior, with shape `[D, 2]`.
        """
        if not "visual_features" in context:
            raise ValueError("Missing `visual_features` keyword argument.")
        batch_size = context["visual_features"].shape[0]
        # Sets initial sample to base distribution's mean.
        x = self._decoder._base_dist.sample(torch.Size([batch_size])).clone().detach().view(
            batch_size, *self._output_shape)
        x.requires_grad = True
        # The contextual parameters, caches for efficiency.
        z = self._params(**context)
        # Initialises a gradient-based optimiser.
        p = [Variable(x[i], requires_grad=True) for i in range(x.shape[0])]
        optimizer = optim.Adam(params=p, lr=lr)
        # Stores the best values.
        x_best = x.clone()
        loss_best = torch.ones((batch_size)).to(x.device) * 1000.0  # pylint: disable=no-member
        for _ in range(num_steps):
            # Resets optimizer's gradients.
            optimizer.zero_grad()
            y, _ = self._decoder._forward(x=torch.stack(p), z=z)
            # Calculates imitation prior.
            _, log_prob, logabsdet = self._decoder._inverse(y=y, z=z)
            imitation_prior = log_prob - logabsdet  # pylint: disable=no-member q(x|phi)
            # Calculates goal likelihodd.
            goal_likelihood = 0.0
            if goal is not None:
                goal_likelihood = self._goal_likelihood(
                    y=y,
                    goal=goal,
                    epsilon=epsilon,
                    comp=1
                )
                assert imitation_prior.shape == goal_likelihood.shape
            loss = -(imitation_prior + goal_likelihood)
            # Backward pass.
            loss.mean().backward(retain_graph=True)
            # Performs a gradient descent step.
            optimizer.step()
            # Book-keeping
            x_best = torch.where(loss.view(-1, 1, 1) < loss_best.view(-1, 1, 1), x, x_best).clone()
            loss_best = torch.where(loss < loss_best, loss, loss_best).clone()
        y, _ = self._decoder._forward(x=x_best, z=z)
        return y
