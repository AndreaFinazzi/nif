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
from oatomobile.torch.loggers import TensorBoardLogger, TensorBoardNegativesLogger
from oatomobile.torch.savers import Checkpointer
from oatomobile.baselines.torch.dim.utility import determine_noise_scale
from torchcontrib.optim import SWA
from preprocessing.aug import augmentation_pipeline
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
    default=False,
    help="If True it clips the gradients norm to 1.0.",
)
flags.DEFINE_bool(
    name="include_depth",
    default=True,
    help="If include depth 4 channels. If not 3 channels.",
)
flags.DEFINE_integer(
    name="num_pos_dim",
    default=3,
    help="The number of position dimensions. 2D or 3D",
)
flags.DEFINE_integer(
    name="past_steps",
    default=10,
    help="The number of past timestep points to expect"
)
flags.DEFINE_bool(
    name="include_lidar",
    default=False,
    help="If true, include a secondary encoder for lidar map input.",
)
flags.DEFINE_bool(
    name="include_rgb",
    default=True,
    help="If true, include 3-channel color input.",
)
flags.DEFINE_bool(
    name="use_swa",
    default=False,
    help="Stochastic Weight Averaging",
)
flags.DEFINE_bool(
    name="use_negative_examples",
    default=False,
    help="Whether to use negative trajectories to tune the model"
)
flags.DEFINE_string(
    name="load_model_path",
    default=None,
    help="Path to loaded model for warm started training"
)
flags.DEFINE_bool(
    name="channels_last",
    default=False,
    help="Whether to put channels first or last in image data."
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
    use_swa = FLAGS.use_swa
    past_steps = FLAGS.past_steps
    use_negs = FLAGS.use_negative_examples
    load_model_path = FLAGS.load_model_path
    include_lidar = FLAGS.include_lidar
    include_rgb = FLAGS.include_rgb

    dataformat = "HWC" if FLAGS.channels_last else "CHW"
    print("Getting empirical noise")
    # try:
    #    noise_level = determine_noise_scale(os.path.join(dataset_dir, "train"))
    #    print("Found valid noise level:", noise_level)
    # except:
    #    noise_level = 0.009

    print("defaulting noise level")
    noise_level = 0.02

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
    input_shape = [past_steps, num_pos_dim]
    model = ImitativeModel(
        output_shape=output_shape,
        include_depth=include_depth,
        num_pos_dim=num_pos_dim,
        input_shape=input_shape,
        include_rgb=include_rgb,
        include_lidar=include_lidar).to(device)

    if load_model_path is not None:
        model.load_state_dict(torch.load(load_model_path))

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
    if use_negs:
        writer = TensorBoardNegativesLogger(log_dir=log_dir, device=device)
    else:
        writer = TensorBoardLogger(log_dir=log_dir, device=device)
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

    def transform(
            batch: Mapping[str, types.Array],
            training: bool = False,
            dataformat: str = dataformat,) -> Mapping[str, torch.Tensor]:
        """Preprocesses a batch for the model.

        Args:
          batch: (keyword arguments) The raw batch variables.

        Returns:
          The processed batch.
        """
        # If training, use random augmentation on each batch
        # MANUALLY REMOVING AUG
        
        #batch = augmentation_pipeline(
        #    {key: tensor.numpy() for (key, tensor) in batch.items()},
        #    training=training,
        #    dataformat=dataformat)

        # Sends tensors to `device`.
        # batch = {key: torch.from_numpy(tensor).to(device) for (key, tensor) in batch.items()}
         
        batch = {key: tensor.type(torch.FloatTensor).to(device) for (key, tensor) in batch.items()}
        
        # Preprocesses batch for the model.
        batch = model.transform(batch)
        return batch

    # Sets up the dataset and the dataloader.

    modalities = ["player_past", "player_future"]
    if include_rgb:
        modalities.append("image_front_color")
    if include_depth:
        modalities.append("image_front_depth")
    if use_negs:
        modalities.append("collides")
    if include_lidar:
        modalities.append("overhead_features")
    modalities = tuple(modalities)

    dataset_train = CARLADataset.as_torch(
        dataset_dir=os.path.join(dataset_dir, "train"),
        modalities=modalities,
    )

    dataloader_train = torch.utils.data.DataLoader(
        dataset_train,
        batch_size=batch_size,
        shuffle=True,
        num_workers=8,
    )
    dataset_val = CARLADataset.as_torch(
        dataset_dir=os.path.join(dataset_dir, "val"),
        modalities=modalities,
    )
    dataloader_val = torch.utils.data.DataLoader(
        dataset_val,
        batch_size=batch_size * 5,
        shuffle=True,
        num_workers=8,
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
        if (include_rgb or include_depth) and include_lidar:
            z = model._params(
                visual_features=batch["visual_features"],
                player_past=batch["player_past"],
                lidar_features=batch["lidar_features"]
            )
        elif include_lidar:
            z = model._params(
                lidar_features=batch["lidar_features"],
                player_past=batch["player_past"]
            )
        else:
            z = model._params(
                visual_features=batch["visual_features"],
                player_past=batch["player_past"]
            )
        
        _, log_prob, logabsdet = model._decoder._inverse(y=y, z=z)

        # Calculates loss (NLL).

        l = log_prob - logabsdet

        if use_negs:
            # print(batch["collides"])
            # batch["collision"] is a boolean array of whether collisions are bad (true) or not (false)
            coeff = torch.where(batch["collides"].byte(), -torch.ones((1)).to(device), torch.ones((1)).to(device))
            # print(torch.where(batch["collides"].byte(), torch.zeros((1)).to(device), torch.ones((1)).to(device)).nonzero(as_tuple=True))
            # print(l.shape)
            neg_loss = -torch.mean(l[torch.where(batch["collides"].byte(), torch.zeros((1)).to(device),
                                   torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]], dim=0)
            pos_loss = torch.mean(l[torch.where(batch["collides"].byte(), torch.ones((1)).to(device),
                                  torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]], dim=0)
            loss = coeff * l
        loss = -torch.mean(l, dim=0)
        # Backward pass.
        loss.backward()

        # Clips gradients norm.
        if clip:
            torch.nn.utils.clip_grad_norm(model.parameters(), 1.0)

        # Performs a gradient descent step.
        optimizer.step()
        if use_negs:
            return neg_loss, pos_loss
        return loss

    def train_epoch(
        model: ImitativeModel,
        optimizer: optim.Optimizer,
        dataloader: torch.utils.data.DataLoader,
    ) -> torch.Tensor:
        """Performs an epoch of gradient descent optimization on `dataloader`."""
        model.train()
        if use_negs:
            pos_loss = 0.0
            neg_loss = 0.0
        else:
            loss = 0.0
        with tqdm.tqdm(dataloader) as pbar:
            for batch in pbar:
                # Prepares the batch.
                batch = transform(batch, training=True)
                # Performs a gradient-descent step.
                if use_negs:
                    losses = train_step(model, optimizer, batch, clip=clip_gradients)
                    neg_loss += losses[0]
                    pos_loss += losses[1]
                else:
                    loss += train_step(model, optimizer, batch, clip=clip_gradients)

        if use_negs:
            return neg_loss / len(dataloader), pos_loss / len(dataloader)

        return loss / len(dataloader)

    def evaluate_step(
        model: ImitativeModel,
        batch: Mapping[str, torch.Tensor],
    ) -> torch.Tensor:
        """Evaluates `model` on a `batch`."""
        # Forward pass from the model.
        if (include_rgb or include_depth) and include_lidar:
            z = model._params(
                visual_features=batch["visual_features"],
                player_past=batch["player_past"],
                lidar_features=batch["lidar_features"]
            )
        elif include_lidar:
            z = model._params(
                lidar_features=batch["lidar_features"],
                player_past=batch["player_past"]
            )
        else:
            z = model._params(
                visual_features=batch["visual_features"],
                player_past=batch["player_past"]
            )

        _, log_prob, logabsdet = model._decoder._inverse(
            y=batch["player_future"][..., :3],
            z=z,
        )

        l = log_prob - logabsdet

        if use_negs:
            neg_loss = -torch.mean(l[torch.where(batch["collides"].byte(), torch.zeros((1)).to(device),
                                   torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]], dim=0)
            pos_loss = torch.mean(l[torch.where(batch["collides"].byte(), torch.ones((1)).to(device),
                                  torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]], dim=0)

        k = 5
        if use_negs:
            mhats_good = []
            mhats_bad = []
        else:
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
            if use_negs:
                bads = batch["player_future"][torch.where(batch["collides"].byte(), torch.ones((1)).to(device),
                                                          torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]]
                goods = batch["player_future"][torch.where(batch["collides"].byte(), torch.zeros((1)).to(device),
                                                           torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]]
                good_z = z[torch.where(batch["collides"].byte(), torch.zeros((1)).to(device), torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]]
                bad_z = z[torch.where(batch["collides"].byte(), torch.ones((1)).to(device), torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]]
                mhat_good, path = model._decoder.m_hat_eval(
                    y=goods[..., :3],
                    z=good_z,
                    k=k
                )
                mhat_bad, path = model._decoder.m_hat_eval(
                    y=bads[..., :3],
                    z=bad_z,
                    k=k
                )
                mhats_good.append(mhat_good)
                mhats_bad.append(mhat_bad)
            else:
                mhat, path = model._decoder.m_hat_eval(
                    y=batch["player_future"][..., :3],
                    z=z,
                    k=k
                )
                mhats.append(mhat)

            k += 5
            paths.append(path)

        # Calculates loss (NLL).

        if use_negs:
            return neg_loss, pos_loss, mhats_good, mhats_bad, np.array(paths[0]), mselist
        else:
            loss = -torch.mean(l, dim=0)  # pylint: disable=no-member
            return loss, mhats, np.array(paths[0]), mselist

    def evaluate_epoch(
        model: ImitativeModel,
        dataloader: torch.utils.data.DataLoader,
    ) -> torch.Tensor:
        """Performs an evaluation of the `model` on the `dataloader."""
        model.eval()
        if use_negs:
            neg_loss = 0.0
            pos_loss = 0.0
            mhat_good = [0.0 for i in range(4)]
            mhat_bad = [0.0 for i in range(4)]
        else:
            loss = 0.0
            mhats = [0.0 for i in range(4)]
        mselist = [0.0 for i in range(10)]
        with tqdm.tqdm(dataloader) as pbar:
            for batch in pbar:
                # Prepares the batch.
                batch = transform(batch)
                # Accumulates loss in dataset.
                with torch.no_grad():
                    if use_negs:
                        n_l, p_l, g_m, b_m, paths, mse = evaluate_step(model, batch)
                        neg_loss, pos_loss, mhat_good, mhat_bad, mselist = neg_loss + n_l, pos_loss + p_l, [mhat_good[i] + g_m[i] for i in range(len(mhat_good))],\
                            [mhat_bad[i] + b_m[i] for i in range(len(mhat_bad))], [mselist[i] + mse[i] for i in range(len(mselist))]
                    else:
                        l, m, paths, mse = evaluate_step(model, batch)
                        loss, mhats, mselist = loss + l, [mhats[i] + m[i] for i in range(len(mhats))],\
                            [mselist[i] + mse[i] for i in range(len(mselist))]

        if use_negs:
            return neg_loss / len(dataloader), pos_loss / len(dataloader), [x / len(dataloader) for x in mhat_good],\
                [x / len(dataloader) for x in mhat_bad], paths, [x / len(dataloader) for x in mselist]
        return loss / len(dataloader), [x / len(dataloader) for x in mhats], paths, [x / len(dataloader) for x in mselist]

    def write(
        model: ImitativeModel,
        dataloader: torch.utils.data.DataLoader,
        writer: TensorBoardLogger,
        split: str,
        loss=None,
        pos_loss=None,
        neg_loss=None,
        mhats=None,
        mhats_exp=None,
        mhats_bad=None,
        paths=None,
        epoch=0,
        mselist=None,
    ) -> None:
        """Visualises model performance on `TensorBoard`."""
        # Gets a sample from the dataset.
        batch = next(iter(dataloader))
        # Prepares the batch.
        batch = transform(batch, training=split == "train")

        # Turns off gradients for model parameters.
        for params in model.parameters():
            params.requires_grad = False

        # Generates predictions.
        predictions = model(num_steps=20, **batch).detach()[:8]
        predictions = predictions.reshape((8, output_shape[0], output_shape[1]))
        past = batch["player_past"][:8].reshape((8, input_shape[0], output_shape[1]))
        if use_negs:
            exp_traj_preds = predictions[torch.where(batch["collides"][:8].byte(), torch.zeros(
                (1)).to(device), torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]].cpu().numpy()
            bad_traj_preds = predictions[torch.where(batch["collides"][:8].byte(), torch.ones(
                (1)).to(device), torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]].cpu().numpy()
            exp_past = batch["player_past"][torch.where(batch["collides"][:8].byte(), torch.zeros((1)).to(device),
                                                        torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]]
            bad_past = batch["player_past"][torch.where(batch["collides"][:8].byte(), torch.ones((1)).to(device),
                                                        torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]]
        else:
            predictions = predictions.cpu().numpy()
            past = batch["player_past"][:8].reshape((8, input_shape[0], output_shape[1]))

        num_samps = 3
        z = model._params(**batch)[:8]
        z = z.repeat((num_samps, 1))
        samples = model._decoder(z).reshape(8, num_samps, output_shape[0], output_shape[1])

        if use_negs:
            exp_samps = samples[torch.where(batch["collides"][:8].byte(), torch.zeros((1)).to(device), torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]]
            bad_samps = samples[torch.where(batch["collides"][:8].byte(), torch.ones((1)).to(device), torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]]
        # msepredictions = model(num_steps=20, goal=batch["player_future"][:, 9, :3].unsqueeze(dim=1), **batch)
        # ga_mse = torch.mean(model.mse_eval(msepredictions, batch["player_future"][:, :, :3])).item()
        ga_mse = 0
        # Turns on gradients for model parameters.
        for params in model.parameters():
            params.requires_grad = True
        lidar = None
        if "lidar_features" in batch:
            lidar = batch["lidar_features"].detach()[:8].cpu().numpy()
        # Logs on `TensorBoard`.
        if "visual_features" in batch:
            ovhf = batch["visual_features"].detach()[:8]
        else:
            ovhf = None
        gt = batch["player_future"].detach()[:8]
        if use_negs:
            if ovhf is not None:
                ofb = ovhf[torch.where(batch["collides"][:8].byte(), torch.ones((1)).to(device), torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]].cpu().numpy()
                ofe = ovhf[torch.where(batch["collides"][:8].byte(), torch.zeros((1)).to(device), torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]].cpu().numpy()
            else:
                ofb = None
                ofe = None
            gte = gt[torch.where(batch["collides"][:8].byte(), torch.zeros((1)).to(device), torch.ones((1)).to(device)).nonzero(as_tuple=True)[0]]
            gtb = gt[torch.where(batch["collides"][:8].byte(), torch.ones((1)).to(device), torch.zeros((1)).to(device)).nonzero(as_tuple=True)[0]]
            writer.posneglog(
                split=split,
                loss_pos=pos_loss,
                loss_neg=neg_loss,
                mhat_bad=mhats_bad,
                mhat_exp=mhats_exp,
                paths=paths,
                image_features_exp=ofe,
                image_features_bad=ofb,
                predictions_exp=exp_samps.cpu().numpy(),
                predictions_bad=bad_samps.cpu().numpy(),
                exp_past=exp_past.detach().cpu().numpy(),
                bad_past=bad_past.detach().cpu().numpy(),
                gt_exp=gte.cpu().numpy(),
                gt_bad=gtb.cpu().numpy(),
                global_step=epoch,
                multiple=True,
                show_depth=include_depth,
                show_lidar=include_lidar,
                show_rgb=include_rgb,
                lidar=lidar
            )
        else:
            if ovhf is not None:
                ovhf = ovhf.cpu().numpy()
            writer.log(
                split=split,
                loss=loss.detach().cpu().numpy().item(),
                mhat=mhats,
                paths=paths,
                image_features=ovhf,
                # predictions=predictions.detach().cpu().numpy()[:8],
                predictions=samples.detach().cpu().numpy(),
                # extrinsics=batch["image_front_extrinsics"],
                extrinsics=None,
                past=past.detach().cpu().numpy(),
                ground_truth=gt.cpu().numpy(),
                global_step=epoch,
                mselist=mselist,
                ga_mse=ga_mse,
                multiple=True,
                show_depth=include_depth,
                show_rgb=include_rgb,
                show_lidar=include_lidar,
                lidar=lidar
            )

    with tqdm.tqdm(range(num_epochs)) as pbar_epoch:
        for epoch in pbar_epoch:
            # Trains model on whole training dataset, and writes on `TensorBoard`.
            if use_negs:
                neg_loss_train, pos_loss_train = train_epoch(model, optimizer, dataloader_train)
                write(model, dataloader_train, writer, "train", None, pos_loss_train, neg_loss_train, None, None, None, None, epoch, None)

                neg_loss_val, pos_loss_val, mhat_val_good, mhat_val_bad, paths, mselist = evaluate_epoch(model, dataloader_val)
                write(model, dataloader_val, writer, "val", None, pos_loss_val, neg_loss_val, None,
                      mhat_val_good, mhat_val_bad, paths[:, 0:8].transpose(1, 0, 2, 3), epoch, mselist)
                if use_swa and epoch + 1 == num_epochs:
                    optimizer.swap_swa_sgd()
                    checkpointer.save(epoch)
                # Checkpoints model weights.
                elif epoch % save_model_frequency == 0:
                    checkpointer.save(epoch)

            else:
                loss_train = train_epoch(model, optimizer, dataloader_train)
                write(model, dataloader_train, writer, "train", loss=loss_train, epoch=epoch)

                # Evaluates model on whole validation dataset, and writes on `TensorBoard`.
                loss_val, mhat_val, paths, mselist = evaluate_epoch(model, dataloader_val)

                write(model, dataloader_val, writer, "val", loss=loss_val, mhats=mhat_val,
                      paths=paths[:, 0:8].transpose(1, 0, 2, 3), epoch=epoch, mselist=mselist)

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
