
import math
from typing import Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.distributions as td

try:
    from sklearn.mixture import GaussianMixture
except Exception:
    GaussianMixture = None

# ---- Utilities ----

def find_best_n_mixtures(X: np.ndarray, Y: np.ndarray, gmm_boost: bool=False, k_list=None) -> int:
    """
    Lightweight BIC-based K selection ONCE at the beginning.
    Uses concatenated [X, Y] features if available. Falls back to fixed K=3 if sklearn unavailable.
    """
    if k_list is None:
        k_list = [1, 2, 3, 4, 5]
    if GaussianMixture is None:
        return 3
    XY = np.concatenate([X, Y.reshape(-1, 1)], axis=1) if Y.ndim == 1 else np.concatenate([X, Y], axis=1)
    best_k, best_bic = 3, float("inf")
    for k in k_list:
        try:
            gmm = GaussianMixture(n_components=k, covariance_type="full", max_iter=300, random_state=42)
            gmm.fit(XY)
            bic = gmm.bic(XY)
            if bic < best_bic:
                best_bic, best_k = bic, k
        except Exception:
            continue
    return int(best_k)

# ---- MDN Model ----

class MDN(nn.Module):
    """
    Simple 1D-output MDN: outputs (pi, mu, sigma) for K Gaussians.
    Numerically-stable: single softmax for pi, softplus for sigma.
    """
    def __init__(self, n_gaussians: int, input_neurons: int, hidden_neurons=None, dropout_p: float=0.1):
        super().__init__()
        if hidden_neurons is None:
            hidden_neurons = [200]
        self.n_gaussians = n_gaussians
        self.input_neurons = input_neurons

        layers = []
        layers.append(nn.Linear(input_neurons, 1000))
        layers.append(nn.GELU())
        in_dim = 1000
        for h in hidden_neurons:
            layers.append(nn.Linear(in_dim, h))
            layers.append(nn.GELU())
            if dropout_p and dropout_p > 0.0:
                layers.append(nn.Dropout(p=dropout_p))
            in_dim = h
        self.fc_layers = nn.Sequential(*layers)

        self.mu = nn.Linear(in_dim, n_gaussians)          # 1D output -> K means
        self.var = nn.Linear(in_dim, n_gaussians)         # pre-activation for sigma
        self.pi  = nn.Linear(in_dim, n_gaussians)         # single softmax done in forward

    def forward(self, x: torch.Tensor) -> td.MixtureSameFamily:
        # expect float32
        z = self.fc_layers(x)
        pi = F.softmax(self.pi(z), dim=1)                            # [B, K]
        mu = self.mu(z).view(-1, self.n_gaussians)                   # [B, K]
        sigma = F.softplus(self.var(z)) + 1e-3                       # [B, K]

        mix = td.Categorical(probs=pi)
        comp = td.Normal(loc=mu, scale=sigma)
        like = td.MixtureSameFamily(mixture_distribution=mix, component_distribution=comp)
        return like

# ---- Loss & decoding ----

def mdn_loss_fn(y: torch.Tensor, likelihood: td.MixtureSameFamily) -> torch.Tensor:
    """
    Negative log-likelihood with correct shape handling.
    y: [B] or [B,1]
    """
    if y.dim() > 1:
        y = y.view(-1)
    nll = -likelihood.log_prob(y)
    return nll.mean()

@torch.no_grad()
def decode_mean_or_top2(like: td.MixtureSameFamily, y_scaler=None, top_thresh: float=0.7) -> np.ndarray:
    """
    If max pi >= top_thresh -> MAP mean; else -> top-2 weighted mean. Returns np.ndarray [B].
    If y_scaler provided (sklearn StandardScaler), inverse_transform is applied.
    """
    mu  = like.component_distribution.loc         # [B, K]
    sig = like.component_distribution.scale       # [B, K]
    pi  = like.mixture_distribution.probs         # [B, K]

    topv, topi = torch.topk(pi, k=2, dim=1)
    use_map = topv[:, 0] >= top_thresh
    map_mean = torch.gather(mu, 1, topi[:, :1]).squeeze(1)                                  # [B]
    top2_mean = (topv * torch.gather(mu, 1, topi)).sum(dim=1) / topv.sum(dim=1).clamp_min(1e-8)  # [B]
    out = torch.where(use_map, map_mean, top2_mean).cpu().numpy().reshape(-1, 1)

    if y_scaler is not None:
        out = y_scaler.inverse_transform(out)
    return out.reshape(-1)
