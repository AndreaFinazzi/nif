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
"""Utility classes for logging on TensorBoard."""

import os
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter

from oatomobile.torch import types

COLORS = [
    "#0071bc",
    "#d85218",
    "#ecb01f",
    "#7d2e8d",
    "#76ab2f",
    "#4cbded",
    "#a1132e",
]

class TensorBoardNegativesLogger:
    """A simple `Pytorch`-friendly `TensorBoard` wrapper."""

    def __init__(self, log_dir: str, device) -> None:
        """Constructs a simgple `TensorBoard` wrapper."""
        # Makes sure output directories exist.
        log_dir_train_positives = os.path.join(log_dir, "train", "loss_pos")
        log_dir_train_negatives = os.path.join(log_dir, "train", "loss_neg")
        log_dir_val_positives = os.path.join(log_dir, "val", "loss_pos")
        log_dir_val_negatives = os.path.join(log_dir, "val", "loss_neg")

        log_dir_val_mhat10_pos = os.path.join(log_dir, "val", "mhat10_pos")
        log_dir_val_mhat10_neg = os.path.join(log_dir, "val", "mhat10_neg")

        os.makedirs(log_dir_train_positives, exist_ok=True)
        os.makedirs(log_dir_train_negatives, exist_ok=True)
        os.makedirs(log_dir_val_positives, exist_ok=True)
        os.makedirs(log_dir_val_negatives, exist_ok=True)
        os.makedirs(log_dir_val_mhat10_pos, exist_ok=True)
        os.makedirs(log_dir_val_mhat10_neg, exist_ok=True)

        self._summary_writter_train_pos = SummaryWriter(log_dir=log_dir_train_positives)
        self._summary_writter_train_neg = SummaryWriter(log_dir=log_dir_train_negatives)
        self._summary_writter_val_pos = SummaryWriter(log_dir=log_dir_val_positives)
        self._summary_writter_val_neg = SummaryWriter(log_dir=log_dir_val_negatives)
        self._summary_writter_mhat_pos = SummaryWriter(log_dir=log_dir_val_mhat10_pos)
        self._summary_writter_mhat_neg = SummaryWriter(log_dir=log_dir_val_mhat10_neg)

    def posneglog(
        self,
        split: str,
        loss_pos,
        loss_neg,
        mhat_bad,
        mhat_exp,
        paths,
        global_step: int,
        image_features_exp,
        image_features_bad,
        predictions_bad,
        predictions_exp,
        exp_past,
        bad_past,
        gt_exp,
        gt_bad,
        extrinsics: Optional[types.Array] = None,
        mselist=None,
        multiple=False
    ) -> None:
        """Logs the scalar loss and visualizes predictions for qualitative
        inspection."""
        if split == "train":
            summary_writter_pos = self._summary_writter_train_pos
            summary_writter_neg = self._summary_writter_train_neg
        elif split == "val":
            summary_writter_pos = self._summary_writter_val_pos
            summary_writter_neg = self._summary_writter_val_neg
            eval_writter1 = self._summary_writter_mhat_pos
            eval_writter2 = self._summary_writter_mhat_neg
            # print(len(mhat_bad), len(mhat_exp))
            eval_writter1.add_scalar(
                tag="Negative Example MHAT@10",
                scalar_value=mhat_bad[1],
                global_step=global_step,
            )
            eval_writter1.add_scalar(
                tag="Expert Example MHAT@10",
                scalar_value=mhat_exp[1],
                global_step=global_step,
            )
        summary_writter_pos.add_scalar(
            tag="Negative Example Loss",
            scalar_value=loss_pos,
            global_step=global_step,
        )
        summary_writter_neg.add_scalar(
            tag="Expert Example Loss",
            scalar_value=loss_neg,
            global_step=global_step,
        )

        if image_features_exp is not None:
            # Visualizes the predictions.
            # (0,2,3,1)
            ran = False
            image_features = np.transpose(image_features_exp,
                                             (0, 2, 3, 1))  # to NHWC
            raw = list()
            for i, (
                o_t,
                p_t,
                g_t,
                past_t
            ) in enumerate(zip(
                image_features,
                predictions_exp,
                gt_exp,
                exp_past
            )):
                # THIS CHANGED THE PREDICTED POINTS TO MATCH IMAGE SPACE
                # msega += np.linalg.norm(g_t - p_t)
                ran = True
                fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(3.0, 3.0))
                bev_meters = 50.0
                plt.xlim([-bev_meters, bev_meters])
                plt.ylim([-bev_meters, bev_meters])
                # Overhead features.
                ax2.imshow(
                    np.flipud(o_t[:, :, 0:3].astype(int)),
                    extent=(-bev_meters, bev_meters, bev_meters, -bev_meters),
                    # extent=(0, 424, 0, 240),
                    # origin='lower'
                )
                g_t = g_t.T
                ax1.plot(
                    g_t[1],
                    g_t[0],
                    marker="o",
                    markersize=4,
                    color=COLORS[1],
                    alpha=1.0,
                    label="ground truth",
                )

                if multiple:
                    for i in range(p_t.shape[0]):
                        p_t_samp = p_t[i].T
                        ax1.plot(
                            p_t_samp[1],
                            p_t_samp[0],
                            marker="o",
                            markersize=4,
                            color=COLORS[0],
                            alpha=0.75,
                        )
                else:
                    p_t = p_t.T
                    ax1.plot(
                        p_t[1],
                        p_t[0],
                        marker="o",
                        markersize=4,
                        color=COLORS[0],
                        alpha=0.75,
                    )
                past_t = past_t.T
                ax1.plot(
                    past_t[1],
                    past_t[0],
                    marker="o",
                    markersize=4,
                    color=COLORS[2],
                    alpha=0.75,
                )
                # LOOP ADDED ABOVE AS A TEST
                ax1.legend()
                ax1.set(frame_on=False)
                ax1.get_xaxis().set_visible(False)
                ax1.get_yaxis().set_visible(False)
                # Convert `matplotlib` canvas to `NumPy` RGB.
                fig.canvas.draw()
                w, h = fig.canvas.get_width_height()
                buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
                buf.shape = (w, h, 4)
                buf = np.roll(buf, 3, axis=2)
                raw.append(buf)
                plt.close(fig)
                # print("was in loop")
            if ran:
                raw = np.reshape(np.asarray(raw), (-1, w, h, 4))

                summary_writter_neg.add_images(
                    tag="Examples",
                  img_tensor=raw[..., :3],
                  global_step=global_step,
                  dataformats="NHWC",
                )
        if image_features_bad is not None:
            # Visualizes the predictions.
            # (0,2,3,1)
            image_features = np.transpose(image_features_bad,
                                             (0, 2, 3, 1))  # to NHWC
            raw = list()
            ran = False
            for i, (
                o_t,
                p_t,
                g_t,
                past_t
            ) in enumerate(zip(
                image_features,
                predictions_bad,
                gt_bad,
                bad_past
            )):
                ran = True
                fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(3.0, 3.0))
                bev_meters = 50.0
                plt.xlim([-bev_meters, bev_meters])
                plt.ylim([-bev_meters, bev_meters])
                # Overhead features.
                ax2.imshow(
                    np.flipud(o_t[:, :, 0:3].astype(int)),
                    extent=(-bev_meters, bev_meters, bev_meters, -bev_meters),
                    # extent=(0, 424, 0, 240),
                    # origin='lower'
                )
                g_t = g_t.T
                ax1.plot(
                    g_t[1],
                    g_t[0],
                    marker="o",
                    markersize=4,
                    color=COLORS[1],
                    alpha=1.0,
                    label="ground truth",
                )

                if multiple:
                    for i in range(p_t.shape[0]):
                        p_t_samp = p_t[i].T
                        ax1.plot(
                            p_t_samp[1],
                            p_t_samp[0],
                            marker="o",
                            markersize=4,
                            color=COLORS[0],
                            alpha=0.75,
                        )
                else:
                    p_t = p_t.T
                    ax1.plot(
                        p_t[1],
                        p_t[0],
                        marker="o",
                        markersize=4,
                        color=COLORS[0],
                        alpha=0.75,
                    )
                past_t = past_t.T
                ax1.plot(
                    past_t[1],
                    past_t[0],
                    marker="o",
                    markersize=4,
                    color=COLORS[2],
                    alpha=0.75,
                )
                # LOOP ADDED ABOVE AS A TEST
                ax1.legend()
                ax1.set(frame_on=False)
                ax1.get_xaxis().set_visible(False)
                ax1.get_yaxis().set_visible(False)
                # Convert `matplotlib` canvas to `NumPy` RGB.
                fig.canvas.draw()
                w, h = fig.canvas.get_width_height()
                buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
                buf.shape = (w, h, 4)
                buf = np.roll(buf, 3, axis=2)
                raw.append(buf)
                plt.close(fig)
            if ran:
                raw = np.reshape(np.asarray(raw), (-1, w, h, 4))
                summary_writter_neg.add_images(
                    tag="Examples",
                  img_tensor=raw[..., :3],
                  global_step=global_step,
                  dataformats="NHWC",
                )

class TensorBoardLogger:
  """A simple `Pytorch`-friendly `TensorBoard` wrapper."""
  def __init__(
      self,
      log_dir: str,
      device,
      normalized:bool=True,
  ) -> None:
    """Constructs a simgple `TensorBoard` wrapper."""
    self._normalized = normalized
    # Makes sure output directories exist.
    log_dir_train = os.path.join(log_dir, "train", "loss")
    log_dir_val = os.path.join(log_dir, "val", "loss")
    log_dir_val_mhat5 = os.path.join(log_dir, "val", "mhat5")
    log_dir_val_mhat10 = os.path.join(log_dir, "val", "mhat10")
    log_dir_val_mhat15 = os.path.join(log_dir, "val", "mhat15")
    log_dir_val_mhat20 = os.path.join(log_dir, "val", "mhat20")
    """
    log_dir_val_c15 = os.path.join(log_dir, "val", "c15")
    log_dir_val_c25 = os.path.join(log_dir, "val", "c25")
    log_dir_val_c35 = os.path.join(log_dir, "val", "c35")
    log_dir_val_c45 = os.path.join(log_dir, "val", "c45")
    log_dir_val_c55 = os.path.join(log_dir, "val", "c55")
    log_dir_val_c65 = os.path.join(log_dir, "val", "c65")
    log_dir_val_c75 = os.path.join(log_dir, "val", "c75")
    log_dir_val_c85 = os.path.join(log_dir, "val", "c85")
    log_dir_val_c95 = os.path.join(log_dir, "val", "c95")
    log_dir_val_c150 = os.path.join(log_dir, "val", "c150")
    """
        # log_dir_val_ga = os.path.join(log_dir, "val", "ga")

    os.makedirs(log_dir_train, exist_ok=True)
    os.makedirs(log_dir_val, exist_ok=True)
    os.makedirs(log_dir_val_mhat5, exist_ok=True)
    os.makedirs(log_dir_val_mhat10, exist_ok=True)
    os.makedirs(log_dir_val_mhat15, exist_ok=True)
    os.makedirs(log_dir_val_mhat20, exist_ok=True)

    # Initialises the `TensorBoard` writters.
    self._summary_writter_train = SummaryWriter(log_dir=log_dir_train)
    self._summary_writter_val = SummaryWriter(log_dir=log_dir_val)
    self._summary_writter_val_mhat5 = SummaryWriter(log_dir=log_dir_val_mhat5)
    self._summary_writter_val_mhat10 = SummaryWriter(log_dir=log_dir_val_mhat10)
    self._summary_writter_val_mhat15 = SummaryWriter(log_dir=log_dir_val_mhat15)
    self._summary_writter_val_mhat20 = SummaryWriter(log_dir=log_dir_val_mhat20)
    """
    self._summary_writter_val_c15 = SummaryWriter(log_dir=log_dir_val_c15)
    self._summary_writter_val_c25 = SummaryWriter(log_dir=log_dir_val_c25)
    self._summary_writter_val_c35 = SummaryWriter(log_dir=log_dir_val_c35)
    self._summary_writter_val_c45 = SummaryWriter(log_dir=log_dir_val_c45)
    self._summary_writter_val_c55 = SummaryWriter(log_dir=log_dir_val_c55)
    self._summary_writter_val_c65 = SummaryWriter(log_dir=log_dir_val_c65)
    self._summary_writter_val_c75 = SummaryWriter(log_dir=log_dir_val_c75)
    self._summary_writter_val_c85 = SummaryWriter(log_dir=log_dir_val_c85)
    self._summary_writter_val_c95 = SummaryWriter(log_dir=log_dir_val_c95)
    self._summary_writter_val_c150 = SummaryWriter(log_dir=log_dir_val_c150)
    """
        # self._summary_writter_val_ga = SummaryWriter(log_dir=log_dir_val_ga)

  def log(
      self,
      split: str,
      loss: float,
      mhat,
      paths,
      global_step: int,
      extrinsics: Optional[types.Array] = None,
      image_features: Optional[types.Array] = None,
      predictions: Optional[types.Array] = None,
      ground_truth: Optional[types.Array] = None,
      past: Optional[types.Array] = None,
      mselist=None,
      ga_mse=None,
      multiple=False,
      show_depth:bool=True,
      show_lidar:bool=False,
      lidar=None
  ) -> None:
    """Logs the scalar loss and visualizes predictions for qualitative
    inspection."""
    if split == "train":
      summary_writter = self._summary_writter_train
    elif split == "val":
      summary_writter = self._summary_writter_val
      eval_writter1 = self._summary_writter_val_mhat5
      eval_writter2 = self._summary_writter_val_mhat10
      eval_writter3 = self._summary_writter_val_mhat15
      eval_writter4 = self._summary_writter_val_mhat20
      # eval_writter5 = self._summary_writter_val_c15
      # eval_writter6 = self._summary_writter_val_c25
      # eval_writter7 = self._summary_writter_val_c35
      # eval_writter8 = self._summary_writter_val_c45
      # eval_writter9 = self._summary_writter_val_c55
      # eval_writter10 = self._summary_writter_val_c65
      # eval_writter11 = self._summary_writter_val_c75
      # eval_writter12 = self._summary_writter_val_c85
      # eval_writter13 = self._summary_writter_val_c95
      # eval_writter14 = self._summary_writter_val_c150
      # eval_writter15 = self._summary_writter_val_ga
      eval_writter1.add_scalar(
          tag="MHAT@5",
          scalar_value=mhat[0],
          global_step=global_step,
      )
      eval_writter2.add_scalar(
          tag="MHAT@10",
          scalar_value=mhat[1],
          global_step=global_step,
      )
      eval_writter3.add_scalar(
          tag="MHAT@15",
          scalar_value=mhat[2],
          global_step=global_step,
      )
      eval_writter4.add_scalar(
          tag="MHAT@20",
          scalar_value=mhat[3],
          global_step=global_step,
      )
      """
      eval_writter5.add_scalar(
          tag="MSE@15",
          scalar_value=mselist[0],
          global_step=global_step,
      )
      eval_writter8.add_scalar(
          tag="MSE@45",
          scalar_value=mselist[3],
          global_step=global_step,
      )
      eval_writter11.add_scalar(
          tag="MSE@75",
          scalar_value=mselist[6],
          global_step=global_step,
      )
      eval_writter13.add_scalar(
          tag="MSE@95",
          scalar_value=mselist[8],
          global_step=global_step,
      )
      eval_writter14.add_scalar(
          tag="MSE @ 150 Trajectories",
          scalar_value=mselist[-1],
          global_step=global_step,
      )
      """
    else:
      raise ValueError("Unrecognised split={} was passed".format(split))

        # Logs the training loss.

    # ext = extrinsics.cpu().detach().numpy()
    summary_writter.add_scalar(
        tag="loss",
        scalar_value=loss,
        global_step=global_step,
    )
    
    if image_features is not None:
      # Visualizes the predictions.
      #(0,2,3,1)
      image_features = np.transpose(image_features,(0, 2, 3, 1))  # to NHWC
      
      if show_lidar:
        lidar = np.transpose(lidar, (0, 2, 3, 1))
      else:
        lidar = np.zeros(image_features.shape)
        
      raw = list()
      for i, (o_t,p_t,g_t,past_t,l_t) in enumerate(zip(image_features,predictions,ground_truth,past,lidar)):
        image = o_t[:, :, 0:3]
        if not self._normalized:
          image = image.astype(int)
        
        if show_depth and show_lidar:
          depth_img = o_t[:, :, 3]
          lidar_img = l_t
          fig, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(7.0, 7.0))
        elif show_depth:
          depth_img = o_t[:, :, 3]
          fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(6.0, 6.0))
        elif show_lidar:
          lidar_img = l_t
          fig, (ax1, ax2, ax4) = plt.subplots(1, 3, figsize=(6.0, 6.0))
        else:
          fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(3.0, 3.0))
        
        bev_meters = 50.0
        #plt.xlim([-bev_meters, bev_meters])
        #plt.ylim([-bev_meters, bev_meters])
        # Overhead features.
        ax2.imshow(
            image[:,:,0], cmap='gray'
            #extent=(-bev_meters, bev_meters, bev_meters, -bev_meters),
        )
        msega=0
        if show_depth:
          ax3.imshow(
            depth_img
            # extent=(-bev_meters, bev_meters, bev_meters, -bev_meters),
          )
        if show_lidar:
          ax4.imshow(
            lidar_img
            #extent=(-bev_meters, bev_meters, bev_meters, -bev_meters),
          )

        # Ground truth.
        g_t = g_t.T
        ax1.plot(
          g_t[1],
          g_t[0],
          marker="o",
          markersize=4,
          color=COLORS[1],
          alpha=1.0,
          label="ground truth",
        )
        if multiple:
          for i in range(p_t.shape[0]):
            p_t_samp = p_t[i].T
            ax1.plot(
              p_t_samp[1],
              p_t_samp[0],
              marker="o",
              markersize=4,
              color=COLORS[0],
              alpha=0.75,
            )
        else:
          p_t = p_t.T
          ax1.plot(
            p_t[1],
            p_t[0],
            marker="o",
            markersize=4,
            color=COLORS[0],
            alpha=0.75,
          )
        past_t = past_t.T
        ax1.plot(
          past_t[1],
          past_t[0],
          marker="o",
          markersize=4,
          color=COLORS[2],
          alpha=0.75,
        )
        ax1.legend()
        ax1.set(frame_on=False)
        ax1.get_xaxis().set_visible(False)
        ax1.get_yaxis().set_visible(False)
        ax1.axis('equal')
        # Convert `matplotlib` canvas to `NumPy` RGB.
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
        buf.shape = (w, h, 4)
        buf = np.roll(buf, 3, axis=2)
        raw.append(buf)
        plt.close(fig)
      raw = np.reshape(np.asarray(raw), (-1, w, h, 4))
      summary_writter.add_images(
        tag="Examples",
        img_tensor=raw[..., :3],
        global_step=global_step,
        dataformats="NHWC",
      )


class EnsembleTensorBoardLogger:
    """A simple `Pytorch`-friendly `TensorBoard` wrapper."""

    def __init__(
        self,
        log_dir: str,
        device
    ) -> None:
        """Constructs a simgple `TensorBoard` wrapper."""
        # Makes sure output directories exist.
        log_dir_train = os.path.join(log_dir, "train", "loss")
        log_dir_train_kl = os.path.join(log_dir, "train", "kl_loss")
        log_dir_train_uncertainty = os.path.join(log_dir, "train", "uncertainty_loss")

        log_dir_val = os.path.join(log_dir, "val", "loss")
        log_dir_val_kl = os.path.join(log_dir, "val", "kl_loss")
        log_dir_val_uncertainty = os.path.join(log_dir, "val", "uncertainty_loss")

        log_dir_val_mhat5 = os.path.join(log_dir, "val", "mhat5")
        log_dir_val_mhat10 = os.path.join(log_dir, "val", "mhat10")
        log_dir_val_mhat15 = os.path.join(log_dir, "val", "mhat15")
        log_dir_val_mhat20 = os.path.join(log_dir, "val", "mhat20")
        """
    log_dir_val_c15 = os.path.join(log_dir, "val", "c15")
    log_dir_val_c25 = os.path.join(log_dir, "val", "c25")
    log_dir_val_c35 = os.path.join(log_dir, "val", "c35")
    log_dir_val_c45 = os.path.join(log_dir, "val", "c45")
    log_dir_val_c55 = os.path.join(log_dir, "val", "c55")
    log_dir_val_c65 = os.path.join(log_dir, "val", "c65")
    log_dir_val_c75 = os.path.join(log_dir, "val", "c75")
    log_dir_val_c85 = os.path.join(log_dir, "val", "c85")
    log_dir_val_c95 = os.path.join(log_dir, "val", "c95")
    log_dir_val_c150 = os.path.join(log_dir, "val", "c150")
    """
        # log_dir_val_ga = os.path.join(log_dir, "val", "ga")

        os.makedirs(log_dir_train, exist_ok=True)
        os.makedirs(log_dir_train_kl, exist_ok=True)
        os.makedirs(log_dir_train_uncertainty, exist_ok=True)

        os.makedirs(log_dir_val, exist_ok=True)
        os.makedirs(log_dir_val_kl, exist_ok=True)
        os.makedirs(log_dir_val_uncertainty, exist_ok=True)

        os.makedirs(log_dir_val_mhat5, exist_ok=True)
        os.makedirs(log_dir_val_mhat10, exist_ok=True)
        os.makedirs(log_dir_val_mhat15, exist_ok=True)
        os.makedirs(log_dir_val_mhat20, exist_ok=True)

        # Initialises the `TensorBoard` writters.
        self._summary_writter_train = SummaryWriter(log_dir=log_dir_train)
        self._summary_writter_train_kl = SummaryWriter(log_dir=log_dir_train_kl)
        self._summary_writter_train_uncertainty = SummaryWriter(log_dir=log_dir_train_uncertainty)

        self._summary_writter_val = SummaryWriter(log_dir=log_dir_val)
        self._summary_writter_val_kl = SummaryWriter(log_dir=log_dir_val_kl)
        self._summary_writter_val_uncertainty = SummaryWriter(log_dir=log_dir_val_uncertainty)

        self._summary_writter_val_mhat5 = SummaryWriter(log_dir=log_dir_val_mhat5)
        self._summary_writter_val_mhat10 = SummaryWriter(log_dir=log_dir_val_mhat10)
        self._summary_writter_val_mhat15 = SummaryWriter(log_dir=log_dir_val_mhat15)
        self._summary_writter_val_mhat20 = SummaryWriter(log_dir=log_dir_val_mhat20)
        """
    self._summary_writter_val_c15 = SummaryWriter(log_dir=log_dir_val_c15)
    self._summary_writter_val_c25 = SummaryWriter(log_dir=log_dir_val_c25)
    self._summary_writter_val_c35 = SummaryWriter(log_dir=log_dir_val_c35)
    self._summary_writter_val_c45 = SummaryWriter(log_dir=log_dir_val_c45)
    self._summary_writter_val_c55 = SummaryWriter(log_dir=log_dir_val_c55)
    self._summary_writter_val_c65 = SummaryWriter(log_dir=log_dir_val_c65)
    self._summary_writter_val_c75 = SummaryWriter(log_dir=log_dir_val_c75)
    self._summary_writter_val_c85 = SummaryWriter(log_dir=log_dir_val_c85)
    self._summary_writter_val_c95 = SummaryWriter(log_dir=log_dir_val_c95)
    self._summary_writter_val_c150 = SummaryWriter(log_dir=log_dir_val_c150)
    """
        # self._summary_writter_val_ga = SummaryWriter(log_dir=log_dir_val_ga)

    def log(
        self,
        split: str,
        loss: float,
        kl_loss: float,
        uncertainty_loss: float,
        mhat,
        paths,
        global_step: int,
        extrinsics: Optional[types.Array] = None,
        image_features: Optional[types.Array] = None,
        predictions: Optional[types.Array] = None,
        ground_truth: Optional[types.Array] = None,
        past: Optional[types.Array] = None,
        mselist=None,
        ga_mse=None,
        multiple:bool=False,
        shoe_depth:bool=True,
    ) -> None:
        """Logs the scalar loss and visualizes predictions for qualitative
        inspection."""
        if split == "train":
            summary_writter = self._summary_writter_train
            summary_writter_kl = self._summary_writter_train_kl
            summary_writter_uncertainty = self._summary_writter_train_uncertainty
        elif split == "val":
            summary_writter = self._summary_writter_val
            summary_writter_kl = self._summary_writter_val_kl
            summary_writter_uncertainty = self._summary_writter_val_uncertainty
            eval_writter1 = self._summary_writter_val_mhat5
            eval_writter2 = self._summary_writter_val_mhat10
            eval_writter3 = self._summary_writter_val_mhat15
            eval_writter4 = self._summary_writter_val_mhat20
            # eval_writter5 = self._summary_writter_val_c15
            # eval_writter6 = self._summary_writter_val_c25
            # eval_writter7 = self._summary_writter_val_c35
            # eval_writter8 = self._summary_writter_val_c45
            # eval_writter9 = self._summary_writter_val_c55
            # eval_writter10 = self._summary_writter_val_c65
            # eval_writter11 = self._summary_writter_val_c75
            # eval_writter12 = self._summary_writter_val_c85
            # eval_writter13 = self._summary_writter_val_c95
            # eval_writter14 = self._summary_writter_val_c150
            # eval_writter15 = self._summary_writter_val_ga
            eval_writter1.add_scalar(
                tag="MHAT@5",
                scalar_value=mhat[0],
                global_step=global_step,
            )
            eval_writter2.add_scalar(
                tag="MHAT@10",
                scalar_value=mhat[1],
                global_step=global_step,
            )
            eval_writter3.add_scalar(
                tag="MHAT@15",
                scalar_value=mhat[2],
                global_step=global_step,
            )
            eval_writter4.add_scalar(
                tag="MHAT@20",
                scalar_value=mhat[3],
                global_step=global_step,
            )
            """
      eval_writter5.add_scalar(
          tag="MSE@15",
          scalar_value=mselist[0],
          global_step=global_step,
      )
      eval_writter8.add_scalar(
          tag="MSE@45",
          scalar_value=mselist[3],
          global_step=global_step,
      )
      eval_writter11.add_scalar(
          tag="MSE@75",
          scalar_value=mselist[6],
          global_step=global_step,
      )
      eval_writter13.add_scalar(
          tag="MSE@95",
          scalar_value=mselist[8],
          global_step=global_step,
      )
      eval_writter14.add_scalar(
          tag="MSE @ 150 Trajectories",
          scalar_value=mselist[-1],
          global_step=global_step,
      )
      """
        else:
            raise ValueError("Unrecognised split={} was passed".format(split))

        # Logs the training loss.

        # ext = extrinsics.cpu().detach().numpy()
        summary_writter.add_scalar(
            tag="loss",
            scalar_value=loss,
            global_step=global_step,
        )
        summary_writter_kl.add_scalar(
            tag="kl_loss",
            scalar_value=kl_loss,
            global_step=global_step,
        )
        summary_writter_uncertainty.add_scalar(
            tag="uncertainty_loss",
            scalar_value=uncertainty_loss,
            global_step=global_step,
        )
        msega = 0
        if image_features is not None:
            # Visualizes the predictions.
            # (0,2,3,1)
            image_features = np.transpose(image_features,
                                             (0, 2, 3, 1))  # to NHWC
            raw = list()
            for i, (
                o_t,
                p_t,
                g_t,
                past_t
            ) in enumerate(zip(
                image_features,
                predictions,
                ground_truth,
                past
            )):
                # THIS CHANGED THE PREDICTED POINTS TO MATCH IMAGE SPACE
                # msega += np.linalg.norm(g_t - p_t)

                # g_t = self.convert_to_image_space(ext[i], g_t.T)
                # g_t = g_t[:, g_t.min(axis=0) >= 0]
                fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(3.0, 3.0))
                bev_meters = 50.0
                plt.xlim([-bev_meters, bev_meters])
                plt.ylim([-bev_meters, bev_meters])
                # Overhead features.

                ax2.imshow(
                    np.flipud(o_t[:, :, 0:3].astype(int)),
                    extent=(-bev_meters, bev_meters, bev_meters, -bev_meters),
                    # extent=(0, 424, 0, 240)
                )

                # Ground truth.
                g_t = g_t.T
                ax1.plot(
                    g_t[1],
                    g_t[0],
                    marker="o",
                    markersize=4,
                    color=COLORS[1],
                    alpha=1.0,
                    label="ground truth",
                )
                # Model prediction.
                # p = self.convert_to_image_space(ext[i], p_t.T)
                # p = p_t[:, p_t.min(axis=0) >= 0]
                # print(p_t, g_t)
                if multiple:
                    for i in range(p_t.shape[0]):
                        p_t_samp = p_t[i].T
                        ax1.plot(
                            p_t_samp[1],
                            p_t_samp[0],
                            marker="o",
                            markersize=4,
                            color=COLORS[0],
                            alpha=0.75,
                        )
                else:
                    p_t = p_t.T
                    ax1.plot(
                        p_t[1],
                        p_t[0],
                        marker="o",
                        markersize=4,
                        color=COLORS[0],
                        alpha=0.75,
                    )
                past_t = past_t.T
                ax1.plot(
                    past_t[1],
                    past_t[0],
                    marker="o",
                    markersize=4,
                    color=COLORS[2],
                    alpha=0.75,
                )
                # LOOP ADDED ABOVE AS A TEST
                ax1.legend()
                ax1.set(frame_on=False)
                ax1.get_xaxis().set_visible(False)
                ax1.get_yaxis().set_visible(False)
                # Convert `matplotlib` canvas to `NumPy` RGB.
                fig.canvas.draw()
                w, h = fig.canvas.get_width_height()
                buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
                buf.shape = (w, h, 4)
                buf = np.roll(buf, 3, axis=2)
                raw.append(buf)
                plt.close(fig)
            raw = np.reshape(np.asarray(raw), (-1, w, h, 4))
            """
      if split == "val":
          eval_writter15.add_scalar(
              tag="MSE @ Gradient Ascent",
              scalar_value=ga_mse,
              global_step=global_step,
          )
      """
            summary_writter.add_images(
                tag="Examples",
                img_tensor=raw[..., :3],
                global_step=global_step,
                dataformats="NHWC",
            )

    def convert_to_image_space(self, extrinsic, predicted_projections):
        predicted_projections[2, :] = 0.55

        intrinsic = np.array(
            [[306.1536560058594, 0.0, 220.7039031982422], [0.0, 306.1067199707031, 125.14686584472656], [0.0, 0.0, 1.0]]
        )

        # intrinsic = np.array(
        #    [[305.5704345703125, 0.0, 219.14657592773438], [0.0, 305.5833435058594, 124.91389465332031], [0.0, 0.0, 1.0]])

        predicted_projections[[0, 2]] = predicted_projections[[2, 0]]
        predicted_projections[[0, 1]] = predicted_projections[[1, 0]]
        predicted_projections[1, :] *= -1
        A = np.dot(intrinsic, extrinsic[0:3, :])
        ppt = np.concatenate((predicted_projections, np.ones((1, predicted_projections.shape[1]))), axis=0)
        img_space = np.dot(A, ppt)
        img_space[0:3, :] = img_space[0:3, :] / (img_space[2, :])
        points = img_space[0:3, :].reshape((3, predicted_projections.shape[1]))
        points[1, :] -= 25
        return points