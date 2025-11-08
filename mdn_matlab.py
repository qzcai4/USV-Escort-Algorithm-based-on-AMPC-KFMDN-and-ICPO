
from array import array
from typing import Tuple
import numpy as np
import torch
import joblib

from mdn_head import MDN, mdn_loss_fn, decode_mean_or_top2

# ---------- Paths & IO helpers ----------

def _model_paths(label: str) -> Tuple[str, str, str]:
    """
    Returns (model_pth, x_scaler_pkl, y_scaler_pkl) under model/{label}/
    Also expects a config text at model/{label}/{label}.txt with n_gaussians and n_input lines.
    """
    base = f"model/{label}/{label}"
    return f"{base}.pth", f"{base}_x_scaler.pkl", f"{base}_y_scaler.pkl"

def _load_config(label: str) -> Tuple[int, int]:
    cfg_path = f"model/{label}/{label}.txt"
    n_gaussians = None
    n_input = None
    with open(cfg_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("n_gaussians"):
                n_gaussians = int(line.split(":")[1])
            if line.startswith("n_input"):
                n_input = int(line.split(":")[1])
    if n_gaussians is None or n_input is None:
        raise RuntimeError(f"Invalid config file: {cfg_path}")
    return n_gaussians, n_input

# ---------- Restore / Predict ----------

def restore_model(label: str) -> MDN:
    n_gaussians, n_input = _load_config(label)
    model_path, _, _ = _model_paths(label)
    model = MDN(n_gaussians=n_gaussians, input_neurons=n_input)
    state = torch.load(model_path, map_location="cpu")
    model.load_state_dict(state)
    model.eval()
    return model

def predict_value(X, label: str):
    """
    MATLAB-friendly prediction entry.
    X can be array.array, list, np.ndarray of shape [B, D] or [D].
    Returns array('d', ...) of predictions in original scale.
    """
    if not isinstance(X, np.ndarray):
        X = np.array(X, dtype=float)

    # reshape
    if X.ndim == 1:
        X = X.reshape(1, -1)

    model_path, x_scaler_path, y_scaler_path = _model_paths(label)
    x_scaler = joblib.load(x_scaler_path)
    y_scaler = joblib.load(y_scaler_path)

    Xn = x_scaler.transform(X)
    model = restore_model(label)
    with torch.no_grad():
        like = model(torch.tensor(Xn, dtype=torch.float32))
        yhat = decode_mean_or_top2(like, y_scaler=y_scaler, top_thresh=0.7)
    return array('d', yhat.tolist())

# Optional: direct sampling from mixture (not used by default)
def sample_from_output(X, label: str, num_samples: int=100):
    if not isinstance(X, np.ndarray):
        X = np.array(X, dtype=float)
    if X.ndim == 1:
        X = X.reshape(1, -1)
    model_path, x_scaler_path, y_scaler_path = _model_paths(label)
    x_scaler = joblib.load(x_scaler_path)
    y_scaler = joblib.load(y_scaler_path)
    Xn = x_scaler.transform(X)
    model = restore_model(label)
    with torch.no_grad():
        like = model(torch.tensor(Xn, dtype=torch.float32))
        mix = like.mixture_distribution.probs      # [B, K]
        mu  = like.component_distribution.loc      # [B, K]
        sig = like.component_distribution.scale    # [B, K]
        B, K = mu.shape
        out = []
        for b in range(B):
            p = mix[b].cpu().numpy()
            k = np.random.choice(K, size=num_samples, p=p)
            samples = np.random.normal(loc=mu[b, k].cpu().numpy(), scale=sig[b, k].cpu().numpy())
            out.append(y_scaler.inverse_transform(samples.reshape(-1,1)).reshape(-1))
    return out
