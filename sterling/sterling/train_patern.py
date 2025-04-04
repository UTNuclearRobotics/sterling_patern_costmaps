import os

import torch
import torch.nn as nn
import torch.nn.functional as F

from sterling.models import (
    CostNet,
    ProprioceptionModel,
    UtilityFuncProprioceptive,
    UtilityFuncVisual,
    VisualEncoderModel,
)


class PaternPreAdaptation(nn.Module):
    def __init__(self, device, pretrained_weights_path=None, latent_size=128):
        super(PaternPreAdaptation, self).__init__()
        self.device = device
        self.latent_size = latent_size  # Fixed at 128D

        # Initialize encoders
        self.visual_encoder = VisualEncoderModel(latent_size=self.latent_size)
        self.proprioceptive_encoder = ProprioceptionModel(latent_size=self.latent_size)
        
        # Utility functions (2-layer MLP on 128D vectors with scaling to 0-255)
        self.uvis = UtilityFuncVisual(latent_size=self.latent_size)
        self.upro = UtilityFuncProprioceptive(latent_size=self.latent_size)
        self.cost_head = CostNet()

        # Load pre-trained weights if provided
        if pretrained_weights_path and os.path.exists(pretrained_weights_path):
            weight_files = {
                "visual_encoder": "fvis.pt",
                "proprioceptive_encoder": "fpro.pt",
                "uvis": "uvis.pt",
                "upro": "upro.pt",
                "cost_head": "cost_head.pt"
            }
            all_files_exist = all(os.path.exists(os.path.join(pretrained_weights_path, file_name)) for file_name in weight_files.values())
            if all_files_exist:
                for submodule_name, file_name in weight_files.items():
                    file_path = os.path.join(pretrained_weights_path, file_name)
                    state_dict = torch.load(file_path, weights_only=True, map_location=device)
                    submodule = getattr(self, submodule_name)
                    submodule.load_state_dict(state_dict)
                    print(f"Loaded {submodule_name} weights from {file_path} for fine-tuning")
            else:
                print(f"Warning: Not all required weight files found in {pretrained_weights_path}. Initializing from scratch.")
        else:
            print(f"No pre-trained weights directory found at {pretrained_weights_path}. Initializing from scratch.")

        # Initialize weights and biases for CostNet layers
        nn.init.kaiming_normal_(self.cost_head.model[0].weight, mode='fan_in', nonlinearity='relu')
        nn.init.constant_(self.cost_head.model[0].bias, 0.0) # First Linear layer
        nn.init.kaiming_normal_(self.cost_head.model[2].weight, mode='fan_in', nonlinearity='relu')
        nn.init.constant_(self.cost_head.model[2].bias, 0.0) # Second Linear layer
        
        self.triplet_loss = nn.TripletMarginLoss(margin=1.0)

    def forward(self, patches, inertial=None):
        patches = patches.to(self.device)
        phi_vis = self.visual_encoder(patches)
        uvis_pred = self.uvis(phi_vis)

        # Optionally process inertial data if provided
        if inertial is not None:
            inertial = inertial.to(self.device)
            phi_pro = self.proprioceptive_encoder(inertial.float())
            upro_pred = self.upro(phi_pro)
        else:
            phi_pro = torch.zeros_like(phi_vis)  # Dummy for consistency
            upro_pred = torch.zeros_like(uvis_pred)  # Dummy for consistency

        # Use only uvis_pred for final cost
        final_cost = self.cost_head(uvis_pred)
        return phi_vis, phi_pro, uvis_pred, upro_pred, final_cost

    def training_step(self, batch, batch_idx):
        patches, inertial, terrain_labels, preferences = batch
        preferences = preferences.to(self.device).float()
        scaled_preferences = self.train_loader.dataset.dataset.get_scaled_preferences(preferences)

        phi_vis, phi_pro, uvis_pred, upro_pred, final_cost = self.forward(patches, inertial)
        
        terrain_labels_tensor = torch.tensor([hash(label) for label in terrain_labels], dtype=torch.long, device=self.device)
        batch_size = len(terrain_labels)
        labels_expanded = terrain_labels_tensor.unsqueeze(1)
        pos_mask = (labels_expanded == labels_expanded.t()) & ~torch.eye(batch_size, dtype=torch.bool, device=self.device)
        neg_mask = (labels_expanded != labels_expanded.t())

        pos_indices = torch.zeros(batch_size, dtype=torch.long, device=self.device)
        neg_indices = torch.zeros(batch_size, dtype=torch.long, device=self.device)
        for i in range(batch_size):
            pos_candidates = pos_mask[i].nonzero(as_tuple=False).flatten()
            neg_candidates = neg_mask[i].nonzero(as_tuple=False).flatten()
            pos_indices[i] = pos_candidates[torch.randint(0, len(pos_candidates), (1,), device=self.device)] if len(pos_candidates) > 0 else i
            neg_indices[i] = neg_candidates[torch.randint(0, len(neg_candidates), (1,), device=self.device)] if len(neg_candidates) > 0 else i

        vis_loss = self.triplet_loss(phi_vis, phi_vis[pos_indices], phi_vis[neg_indices])
        pro_loss = self.triplet_loss(phi_pro, phi_pro[pos_indices], phi_pro[neg_indices])

        pref_diff = scaled_preferences.unsqueeze(1) - scaled_preferences.unsqueeze(0)  # Use scaled preferences
        pred_diff = uvis_pred.unsqueeze(1) - uvis_pred.unsqueeze(0)
        ranking_mask = pref_diff > 0
        ranking_loss = F.relu(1.0 - (pred_diff / 100.0)[ranking_mask]).mean() if ranking_mask.any() else torch.tensor(0.0, device=self.device)

        modality_mse_loss = F.mse_loss(uvis_pred.detach(), upro_pred)
        cost_loss = F.smooth_l1_loss(final_cost, scaled_preferences)

        #total_loss = 1.0 * (vis_loss + 0.1*pro_loss) + 0.5 * ranking_loss + 0.5 * modality_mse_loss + 1.0 * cost_loss
        total_loss = 1.0 * (vis_loss + pro_loss) + 0.5 * ranking_loss + 0.5 * modality_mse_loss + 2.0 * cost_loss

        #print(f"Train Batch {batch_idx}: vis_loss={vis_loss.item():.4f}, pro_loss={pro_loss.item():.4f}, "
        #      f"ranking_loss={ranking_loss.item():.4f}, modality_mse_loss={modality_mse_loss.item():.4f}, "
        #      f"cost_loss={cost_loss.item():.4f}, total_loss={total_loss.item():.4f}")
        #print(f"uvis_pred range: {uvis_pred.min().item():.4f} to {uvis_pred.max().item():.4f}")
        print(f"final_cost range: {final_cost.min().item():.4f} to {final_cost.max().item():.4f}")
        #print(f"scaled_preferences range: {scaled_preferences.min().item():.4f} to {scaled_preferences.max().item():.4f}")
        return total_loss

    def validation_step(self, batch, batch_idx):
        patches, inertial, terrain_labels, preferences = batch
        preferences = preferences.to(self.device).float()
        scaled_preferences = self.val_loader.dataset.dataset.get_scaled_preferences(preferences)

        phi_vis, phi_pro, uvis_pred, upro_pred, final_cost = self.forward(patches, inertial)

        terrain_labels_tensor = torch.tensor([hash(label) for label in terrain_labels], dtype=torch.long, device=self.device)
        batch_size = len(terrain_labels)
        labels_expanded = terrain_labels_tensor.unsqueeze(1)
        pos_mask = (labels_expanded == labels_expanded.t()) & ~torch.eye(batch_size, dtype=torch.bool, device=self.device)
        neg_mask = (labels_expanded != labels_expanded.t())
        pos_indices = torch.zeros(batch_size, dtype=torch.long, device=self.device)
        neg_indices = torch.zeros(batch_size, dtype=torch.long, device=self.device)
        for i in range(batch_size):
            pos_candidates = pos_mask[i].nonzero(as_tuple=False).flatten()
            neg_candidates = neg_mask[i].nonzero(as_tuple=False).flatten()
            pos_indices[i] = pos_candidates[torch.randint(0, len(pos_candidates), (1,), device=self.device)] if len(pos_candidates) > 0 else i
            neg_indices[i] = neg_candidates[torch.randint(0, len(neg_candidates), (1,), device=self.device)] if len(neg_candidates) > 0 else i

        vis_loss = self.triplet_loss(phi_vis, phi_vis[pos_indices], phi_vis[neg_indices])
        pro_loss = self.triplet_loss(phi_pro, phi_pro[pos_indices], phi_pro[neg_indices])

        pref_diff = scaled_preferences.unsqueeze(1) - scaled_preferences.unsqueeze(0)
        pred_diff = uvis_pred.unsqueeze(1) - uvis_pred.unsqueeze(0)
        ranking_mask = pref_diff > 0
        ranking_loss = F.relu(1.0 - (pred_diff / 100.0)[ranking_mask]).mean() if ranking_mask.any() else torch.tensor(0.0, device=self.device)

        modality_mse_loss = F.mse_loss(uvis_pred.detach(), upro_pred)
        cost_loss = F.smooth_l1_loss(final_cost, scaled_preferences)

        #total_loss = 1.0 * (vis_loss + 0.1*pro_loss) + 0.5 * ranking_loss + 0.5 * modality_mse_loss + 1.0 * cost_loss
        total_loss = 1.0 * (vis_loss + pro_loss) + 0.5 * ranking_loss + 0.5 * modality_mse_loss + 2.0 * cost_loss
        return total_loss