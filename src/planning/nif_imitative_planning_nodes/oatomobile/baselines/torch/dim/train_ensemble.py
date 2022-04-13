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
"""Trains the deep imitative model on expert demostrations."""

import os
from typing import Mapping
from typing import Sequence

import torch
import torch.distributions as D
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import tqdm
from absl import app
from absl import flags
from absl import logging
import numpy as np
import os
import sys
sys.path.append(os.getcwd())
from oatomobile.baselines.torch.dim.model import ImitativeModel
from oatomobile.datasets.carla import CARLADataset
from oatomobile.torch import types
from oatomobile.torch.loggers import EnsembleTensorBoardLogger
from oatomobile.torch.savers import Checkpointer
from oatomobile.baselines.torch.dim.utility import determine_noise_scale
from torchcontrib.optim import SWA

logging.set_verbosity(logging.DEBUG)
FLAGS = flags.FLAGS
flags.DEFINE_string(
    name="dataset_dir",
    default=None,
    help="The full path to the processed dataset.",
)
flags.DEFINE_string(
    name="output_dir",
    default=None,
    help="The full path to the output directory (for logs, ckpts).",
)
flags.DEFINE_integer(
    name="batch_size",
    default=32,
    help="The batch size used for training the neural network.",
)
flags.DEFINE_integer(
    name="num_epochs",
    default=None,
    help="The number of training epochs for the neural network.",
)
flags.DEFINE_integer(
    name="save_model_frequency",
    default=4,
    help="The number epochs between saves of the model.",
)
flags.DEFINE_float(
    name="learning_rate",
    default=1e-3,
    help="The ADAM learning rate.",
)
flags.DEFINE_integer(
    name="num_timesteps_to_keep",
    default=10,
    help="The numbers of time-steps to keep from the target, with downsampling.",
)
flags.DEFINE_float(
    name="weight_decay",
    default=0.0,
    help="The L2 penalty (regularization) coefficient.",
)
flags.DEFINE_bool(
    name="clip_gradients",
    default=True,
    help="If True it clips the gradients norm to 1.0.",
)

flags.DEFINE_bool(
    name="include_depth",
    default=False,
    help="If include depth 4 channels. If not 3 channels.",
)

flags.DEFINE_integer(
    name="num_pos_dim",
    default=2,
    help="The number of position dimensions. 2D or 3D",
)

flags.DEFINE_float(
    name="uncertainty_weight",
    default=1.0,
    help="The weight for loss = kl_loss * uncertainty_weight * uncertainty_loss",
)

flags.DEFINE_bool(
    name="use_swa",
    default=False,
    help="Stochastic Weight Averaging",
)


def main(argv):
    # Debugging purposes.
    logging.debug(argv)
    logging.debug(FLAGS)

    # Parses command line arguments.
    dataset_dir = FLAGS.dataset_dir
    output_dir = FLAGS.output_dir
    batch_size = FLAGS.batch_size
    num_epochs = FLAGS.num_epochs
    learning_rate = FLAGS.learning_rate
    save_model_frequency = FLAGS.save_model_frequency
    num_timesteps_to_keep = FLAGS.num_timesteps_to_keep
    weight_decay = FLAGS.weight_decay
    clip_gradients = FLAGS.clip_gradients
    include_depth = FLAGS.include_depth
    num_pos_dim = FLAGS.num_pos_dim
    uncertainty_weight = FLAGS.uncertainty_weight
    use_swa = FLAGS.use_swa

    print("Getting empirical noise")
    noise_level = determine_noise_scale(os.path.join(dataset_dir, "train"))
    print("Found valid noise level:", noise_level)

    # Determines device, accelerator.
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # pylint: disable=no-member

    # Creates the necessary output directory.
    os.makedirs(output_dir, exist_ok=True)
    log_dir = os.path.join(output_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)
    ckpt_dir = os.path.join(output_dir, "ckpts")
    os.makedirs(ckpt_dir, exist_ok=True)

    # Initializes the model and its optimizer.
    # change between 2 and 3 for 2D and 3D..
    output_shape = [num_timesteps_to_keep, num_pos_dim]

    models = [ImitativeModel(output_shape=output_shape, include_depth=include_depth, num_pos_dim=num_pos_dim).to(device) for _ in range(8)]
    ckpts = ['output/ckpts/model-32.pt', 'output/ckpts/model-36.pt',
             'output/ckpts/model-40.pt', 'output/ckpts/model-44.pt',
             'output/ckpts/model-48.pt', 'output/ckpts/model-52.pt',
             'output/ckpts/model-56.pt', 'output/ckpts/model-60.pt']  # Paths to the model checkpoints.
    for model, ckpt in zip(models, ckpts):
        model.load_state_dict(torch.load(ckpt))

#   model = models.pop(0) #assign and remove
    model = ImitativeModel(output_shape=output_shape, include_depth=include_depth, num_pos_dim=num_pos_dim).to(device)

    # Freeze all the parameters in all other models than the first
    # is everything frozen correctly? freeze the decoder?
    for temp_model in models:
        for param in temp_model.parameters():
            param.requires_grad = False
        # make sure _decoder weight is frozen
        # for param in temp_model._decoder.parameters():
        #   print('param.requires_grad', param.requires_grad)
        #   import pdb; pdb.set_trace()

    base_optimizer = optim.Adam(
        model.parameters(),
        lr=learning_rate,
        weight_decay=weight_decay,
    )

    if use_swa:
        # more tunable hyperparameters, SWA(base_opt, swa_start, swa_freq, swa_lr)
        optimizer = SWA(base_optimizer, swa_start=5000, swa_freq=5, swa_lr=0.01)
    else:
        optimizer = base_optimizer

    writer = EnsembleTensorBoardLogger(log_dir=log_dir, device=device)
    checkpointer = Checkpointer(model=model, ckpt_dir=ckpt_dir)
    clusterlist = []

    """
  for i in range(1, 10):
      with open("C:\\Users\\nrdas\\Downloads\\BAIR\\railracer\\trajectory_lib\\RC\\" + str(10 * i + 5) + "_clusters.npy", 'rb') as f:
          arr = np.load(f)
          clusterlist.append(torch.from_numpy(np.reshape(arr, (arr.shape[0], 10, 3))).type(torch.FloatTensor).to(device))

  with open(r"C:\\Users\\nrdas\Downloads\BAIR\railracer\trajectory_lib\RC\150_clusters.npy", 'rb') as f:
      arr = np.load(f)
      clusterlist.append(torch.from_numpy(np.reshape(arr, (arr.shape[0], 10, 3))).type(torch.FloatTensor).to(device))
  """

    def transform(batch: Mapping[str, types.Array]) -> Mapping[str, torch.Tensor]:
        """Preprocesses a batch for the model.

        Args:
          batch: (keyword arguments) The raw batch variables.

        Returns:
          The processed batch.
        """
        # Sends tensors to `device`.
        batch = {key: tensor.to(device) for (key, tensor) in batch.items()}
        # Preprocesses batch for the model.
        batch = model.transform(batch)
        return batch

    # Sets up the dataset and the dataloader.
    if include_depth:
        modalities = (
            "image_front_color",
            "image_front_depth",
            "player_past",
            "player_future",
        )
    else:
        modalities = (
            "image_front_color",
            "player_past",
            "player_future",
        )

    """
  modalities = (
      "image_front_depth",
      "image_front_color",
      "player_past",
      "player_future",
      "image_front_extrinsics"
  )
  """
    dataset_train = CARLADataset.as_torch(
        dataset_dir=os.path.join(dataset_dir, "train"),
        modalities=modalities,
    )

    dataloader_train = torch.utils.data.DataLoader(
        dataset_train,
        batch_size=batch_size,
        shuffle=True,
        num_workers=5,
    )
    dataset_val = CARLADataset.as_torch(
        dataset_dir=os.path.join(dataset_dir, "val"),
        modalities=modalities,
    )
    dataloader_val = torch.utils.data.DataLoader(
        dataset_val,
        batch_size=batch_size * 5,
        shuffle=True,
        num_workers=5,
    )

    # Theoretical limit of NLL.
    nll_limit = -torch.sum(  # pylint: disable=no-member
        D.MultivariateNormal(
            loc=torch.zeros(output_shape[-2] * output_shape[-1]),  # pylint: disable=no-member
            scale_tril=torch.eye(output_shape[-2] * output_shape[-1]) *  # pylint: disable=no-member
            noise_level,  # pylint: disable=no-member
        ).log_prob(torch.zeros(output_shape[-2] * output_shape[-1])))  # pylint: disable=no-member

    def train_step(
        model: ImitativeModel,
        models: Sequence[ImitativeModel],
        optimizer: optim.Optimizer,
        batch: Mapping[str, torch.Tensor],
        clip: bool = False,
    ) -> torch.Tensor:
        """Performs a single gradient-descent optimisation step."""
        # Resets optimizer's gradients.
        optimizer.zero_grad()

        # Perturb target.
        y = torch.normal(  # pylint: disable=no-member
            mean=batch["player_future"][..., :3],
            std=torch.ones_like(batch["player_future"][..., :3]) * noise_level,  # pylint: disable=no-member
        )

        # Forward pass from the model.
        z = model._params(
            visual_features=batch["visual_features"],
            player_past=batch["player_past"]
        )
        _, log_prob, logabsdet = model._decoder._inverse(y=y, z=z)

        # danielshin modify loss here to take in ensemble loss, KL(p, q_final) + E_{traj ~ q_final} u(traj; {q_i})
        # E_{traj ~ q_final} u(traj; {q_i}), take a trajectory sample from q_final, find imitation prior of this trajectory
        # under different q_i, and take variance, repeat process for a bunch of trajectories.

        # 1) take a trajectory sample from q_final
        # Forward-pass, stochastic generation of a sequence.
        y = model._decoder.forward(z=z)  # should be shaped B, so averaged across batch, probably good to have a large batch

        # 2) find imitation prior of this trajectory
        imitation_prior_list = []
        for i in range(len(models)):
            # log_prob, logabsdet with shape `[B]`.
            _, temp_log_prob, temp_logabsdet = models[i]._decoder._inverse(y=y, z=z)  # Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
            imitation_prior = temp_log_prob - temp_logabsdet
            imitation_prior_list.append(imitation_prior)

        # 3) take variance
        # log_prob_list is list of torch.Tensor
        imitation_prior_tensor = torch.stack(imitation_prior_list, dim=0)
        var_tensor = torch.var(imitation_prior_tensor, 0)  # should be shaped B same shape as log_prob, dim could be 0 or 1, double check on this
        # import pdb; pdb.set_trace()
        assert var_tensor.size() == log_prob.size()  # both should be shape [B]

        # mean of the variance
        # Calculates loss (NLL).
        kl_loss = -torch.mean(log_prob - logabsdet, dim=0)
        uncertainty_loss = uncertainty_weight * torch.mean(var_tensor, dim=0)  # pylint: disable=no-member
        loss = kl_loss + uncertainty_loss

        # Backward pass.
        loss.backward()

        # Clips gradients norm.
        if clip:
            torch.nn.utils.clip_grad_norm(model.parameters(), 1.0)

        # Performs a gradient descent step.
        optimizer.step()

        return loss, kl_loss, uncertainty_loss

    def train_epoch(
        model: ImitativeModel,
        models: Sequence[ImitativeModel],
        optimizer: optim.Optimizer,
        dataloader: torch.utils.data.DataLoader,
    ) -> torch.Tensor:
        """Performs an epoch of gradient descent optimization on `dataloader`."""
        model.train()
        loss, kl_loss, uncertainty_loss = 0.0, 0.0, 0.0
        with tqdm.tqdm(dataloader) as pbar:
            for batch in pbar:
                # Prepares the batch.
                batch = transform(batch)
                # Performs a gradient-descent step.
                temp_loss, temp_kl_loss, temp_uncertainty_loss = train_step(model, models, optimizer, batch, clip=clip_gradients)
                loss, kl_loss, uncertainty_loss = loss + temp_loss, kl_loss + temp_kl_loss, uncertainty_loss + temp_uncertainty_loss
        return loss / len(dataloader), kl_loss / len(dataloader), uncertainty_loss / len(dataloader)

    def evaluate_step(
        model: ImitativeModel,
        models: Sequence[ImitativeModel],
        batch: Mapping[str, torch.Tensor],
    ) -> torch.Tensor:
        """Evaluates `model` on a `batch`."""
        # Forward pass from the model.
        z = model._params(
            visual_features=batch["visual_features"],
            player_past=batch["player_past"]
        )
        _, log_prob, logabsdet = model._decoder._inverse(
            y=batch["player_future"][..., :3],
            z=z,
        )
        k = 5
        mhats = []
        paths = []
        mselist = [0 for i in range(10)]
        """
    for cluster in clusterlist:
        avg = 0
        for i in range(batch["player_past"].shape[0]):
            traj, prb = model.trajectory_library_plan(cluster, z[i].repeat((cluster.shape[0], 1)),
                        batch["player_future"][i][9][:3].unsqueeze(dim=0).unsqueeze(dim=0).repeat((cluster.shape[0], 1, 1)))
            avg += model.mse_eval(traj, batch["player_future"][i][..., :3]).item()
        mselist.append(avg / batch["player_past"].shape[0])
        avg = 0
    """
        while k <= 20:
            mhat, path = model._decoder.m_hat_eval(
                y=batch["player_future"][..., :3],
                z=z,
                k=k
            )
            k += 5
            mhats.append(mhat)
            paths.append(path)

        # 1) take a trajectory sample from q_final
        # Forward-pass, stochastic generation of a sequence.
        y = model._decoder.forward(z=z)  # should be shaped B, so averaged across batch, probably good to have a large batch

        # 2) find imitation prior of this trajectory
        imitation_prior_list = []
        for i in range(len(models)):
            # log_prob, logabsdet with shape `[B]`.
            _, temp_log_prob, temp_logabsdet = models[i]._decoder._inverse(y=y, z=z)  # Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
            imitation_prior = temp_log_prob - temp_logabsdet
            imitation_prior_list.append(imitation_prior)

        # 3) take variance
        # log_prob_list is list of torch.Tensor
        imitation_prior_tensor = torch.stack(imitation_prior_list, dim=0)
        var_tensor = torch.var(imitation_prior_tensor, 0)  # should be shaped B same shape as log_prob, dim could be 0 or 1, double check on this
        assert var_tensor.size() == log_prob.size()  # both should be shape [B]

        # mean of the variance
        # Calculates loss (NLL).
        kl_loss = -torch.mean(log_prob - logabsdet, dim=0)
        uncertainty_loss = uncertainty_weight * torch.mean(var_tensor, dim=0)  # pylint: disable=no-member
        loss = kl_loss + uncertainty_loss

        return loss, kl_loss, uncertainty_loss, mhats, np.array(paths[0]), mselist

    def evaluate_epoch(
        model: ImitativeModel,
        models: Sequence[ImitativeModel],
        dataloader: torch.utils.data.DataLoader,
    ) -> torch.Tensor:
        """Performs an evaluation of the `model` on the `dataloader."""
        model.eval()
        loss, kl_loss, uncertainty_loss = 0.0, 0.0, 0.0
        mhats = [0.0 for i in range(4)]
        mselist = [0.0 for i in range(10)]
        with tqdm.tqdm(dataloader) as pbar:
            for batch in pbar:
                # Prepares the batch.
                batch = transform(batch)
                # Accumulates loss in dataset.
                with torch.no_grad():
                    l, kl_l, uncertainty_l, m, paths, mse = evaluate_step(model, models, batch)
                    loss, kl_loss, uncertainty_loss, mhats, mselist = loss + l, kl_loss + kl_l, uncertainty_loss + uncertainty_l,\
                        [mhats[i] + m[i] for i in range(len(mhats))],\
                        [mselist[i] + mse[i] for i in range(len(mselist))]
        return loss / len(dataloader), kl_loss / len(dataloader), uncertainty_loss / len(dataloader), \
            [x / len(dataloader) for x in mhats], paths, [x / len(dataloader) for x in mselist]

    def write(
        model: ImitativeModel,
        dataloader: torch.utils.data.DataLoader,
        writer: EnsembleTensorBoardLogger,
        split: str,
        loss: torch.Tensor,
        kl_loss: torch.Tensor,
        uncertainty_loss: torch.Tensor,
        mhats,
        paths,
        epoch: int,
        mselist
    ) -> None:
        """Visualises model performance on `TensorBoard`."""
        # Gets a sample from the dataset.
        batch = next(iter(dataloader))
        # Prepares the batch.
        batch = transform(batch)
        # Turns off gradients for model parameters.
        for params in model.parameters():
            params.requires_grad = False

        # Generates predictions.
        predictions = model(num_steps=20, **batch).detach().cpu().numpy()[:8]
        predictions = predictions.reshape((8, output_shape[0], output_shape[1]))
        past = batch["player_past"][:8].reshape((8, 10, output_shape[1]))
        num_samps = 3
        z = model._params(**batch)[:8]
        z = z.repeat((num_samps, 1))
        samples = model._decoder(z).reshape(8, num_samps, output_shape[0], output_shape[1])
        # msepredictions = model(num_steps=20, goal=batch["player_future"][:, 9, :3].unsqueeze(dim=1), **batch)
        # ga_mse = torch.mean(model.mse_eval(msepredictions, batch["player_future"][:, :, :3])).item()
        ga_mse = 0
        # Turns on gradients for model parameters.
        for params in model.parameters():
            params.requires_grad = True
        # Logs on `TensorBoard`.
        writer.log(
            split=split,
            loss=loss.detach().cpu().numpy().item(),
            kl_loss=kl_loss.detach().cpu().numpy().item(),
            uncertainty_loss=uncertainty_loss.detach().cpu().numpy().item(),
            mhat=mhats,
            paths=paths,
            overhead_features=batch["visual_features"].detach().cpu().numpy()[:8],
            # predictions=predictions.detach().cpu().numpy()[:8],
            predictions=samples.detach().cpu().numpy(),
            # extrinsics=batch["image_front_extrinsics"],
            extrinsics=None,
            past=past.detach().cpu().numpy(),
            ground_truth=batch["player_future"].detach().cpu().numpy()[:8],
            global_step=epoch,
            mselist=mselist,
            ga_mse=ga_mse,
            multiple=True
        )

    with tqdm.tqdm(range(num_epochs)) as pbar_epoch:
        for epoch in pbar_epoch:
            # Trains model on whole training dataset, and writes on `TensorBoard`.
            loss_train, kl_loss_train, uncertainty_loss_train = train_epoch(model, models, optimizer, dataloader_train)
            write(model, dataloader_train, writer, "train", loss_train, kl_loss_train, uncertainty_loss_train, None, None, epoch, None)

            # Evaluates model on whole validation dataset, and writes on `TensorBoard`.
            loss_val, kl_loss_val, uncertainty_loss_val, mhat_val, paths, mselist = evaluate_epoch(model, models, dataloader_val)

            write(model, dataloader_val, writer, "val", loss_val, kl_loss_val, uncertainty_loss_val,
                  mhat_val, paths[:, 0:8].transpose(1, 0, 2, 3), epoch, mselist)

            if use_swa and epoch + 1 == num_epochs:
                optimizer.swap_swa_sgd()
                checkpointer.save(epoch)
            # Checkpoints model weights.
            elif epoch % save_model_frequency == 0:
                checkpointer.save(epoch)

            # Updates progress bar description.
            pbar_epoch.set_description(
                "TL: {:.2f} | VL: {:.2f} | THEORYMIN: {:.2f}".format(
                    loss_train.detach().cpu().numpy().item(),
                    loss_val.detach().cpu().numpy().item(),
                    nll_limit,
                ))


if __name__ == "__main__":
    flags.mark_flag_as_required("dataset_dir")
    flags.mark_flag_as_required("output_dir")
    flags.mark_flag_as_required("num_epochs")
    app.run(main)
