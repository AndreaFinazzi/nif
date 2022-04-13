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
from oatomobile.baselines.torch.bc.model import BCModel
from oatomobile.datasets.carla import CARLADataset
from oatomobile.torch import types
from oatomobile.torch.loggers import TensorBoardLogger
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
flags.DEFINE_bool(
    name="goal_directed",
    default=False,
    help="Whether to train a model that is goal-directed or not",
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
    goal_directed = FLAGS.goal_directed

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    os.makedirs(output_dir, exist_ok=True)
    log_dir = os.path.join(output_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)
    ckpt_dir = os.path.join(output_dir, "ckpts")
    os.makedirs(ckpt_dir, exist_ok=True)
    
    output_shape = [num_timesteps_to_keep, num_pos_dim]
    
    model = BCModel(output_shape=output_shape, include_depth=include_depth, goal_directed=goal_directed).to(device)
    
    optimizer = optim.Adam(
      model.parameters(),
      lr=learning_rate,
      weight_decay=weight_decay,
    )

    writer = TensorBoardLogger(log_dir=log_dir, device=device)
    checkpointer = Checkpointer(model=model, ckpt_dir=ckpt_dir)
    

    def transform(batch):
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


    def train_step(model, optimizer, batch, clip):
        optimizer.zero_grad()
        traj = model(**batch)
        loss = nn.MSELoss()
        output = loss(traj, batch["player_future"])
        output.backward()
        if clip:
            torch.nn.utils.clip_grad_norm(model.parameters(), 1.0)
        optimizer.step()
        return output.data

    def train_epoch(model, optimizer, dataloader):
        model.train()
        loss = 0.0
        with tqdm.tqdm(dataloader) as pbar:
          for batch in pbar:
            # Prepares the batch.
            batch = transform(batch)
            # Performs a gradient-descent step.
            loss += train_step(model, optimizer, batch, clip=clip_gradients)
        return loss / len(dataloader)

    def evaluate_step(model, batch):
        traj = model(**batch)
        loss = nn.MSELoss()
        output = loss(traj, batch["player_future"])
        return output.data

    def evaluate_epoch(model, dataloader):
        model.eval()
        loss = 0.0
        with tqdm.tqdm(dataloader) as pbar:
          for batch in pbar:
            # Prepares the batch.
            batch = transform(batch)
            # Performs a gradient-descent step.
            loss += evaluate_step(model, batch)
        return loss / len(dataloader)

    def write(model, dataloader, writer, split, loss, epoch):
        batch = next(iter(dataloader))
        # Prepares the batch.
        batch = transform(batch)
        # Turns off gradients for model parameters.
        for params in model.parameters():
            params.requires_grad = False
        predictions = model(**batch).detach().cpu().numpy()[:8]
        past = batch["player_past"][:8].view((8, output_shape[0], output_shape[1]))
        mhats = [0,0,0,0,0,0,0,0,0]
        for params in model.parameters():
            params.requires_grad = True
        writer.log(
            split=split,
            loss=loss.detach().cpu().numpy().item(),
            overhead_features=batch["visual_features"].detach().cpu().numpy()[:8],
            predictions=predictions,
            mhat=mhats,
            paths=None,
            past=past.detach().cpu().numpy(),
            ground_truth=batch["player_future"].detach().cpu().numpy()[:8],
            global_step=epoch,
        )

    with tqdm.tqdm(range(num_epochs)) as pbar_epoch:  
        for epoch in pbar_epoch:
            # Trains model on whole training dataset, and writes on `TensorBoard`.
            loss_train = train_epoch(model, optimizer, dataloader_train)
            write(model, dataloader_train, writer, "train", loss_train, epoch)

            # Evaluates model on whole validation dataset, and writes on `TensorBoard`.
            loss_val = evaluate_epoch(model, dataloader_val)
            write(model, dataloader_val, writer, "val", loss_val, epoch)

            # Checkpoints model weights.
            if epoch % save_model_frequency == 0:
                checkpointer.save(epoch)

            # Updates progress bar description.
            pbar_epoch.set_description(
              "TL: {:.2f} | VL: {:.2f}".format(
                  loss_train.detach().cpu().numpy().item(),
                  loss_val.detach().cpu().numpy().item(),
              ))

if __name__ == "__main__":
    flags.mark_flag_as_required("dataset_dir")
    flags.mark_flag_as_required("output_dir")
    flags.mark_flag_as_required("num_epochs")
    app.run(main)

