# utils.py
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import rospy


def kernel_osc(x, a, b, alpha):
    return a * (np.exp(-b * np.abs(x)) * ((b * np.sin(np.abs(alpha * x))) + np.cos(alpha * x)))
    
def kernel_gauss(x, a_ex, s_ex, w_in):
    return a_ex * np.exp(-0.5 * x**2 / s_ex**2) - w_in


def load_sequence_memory(data_dir, filename=None):
    if filename is None:
        files = [f for f in os.listdir(data_dir) if f.startswith("u_sm_") and f.endswith('.npy')]
        if not files:
            raise IOError("No 'u_sm_' files found in the data folder.")
        filename = max([os.path.join(data_dir, f) for f in files], key=os.path.getmtime)
    data = np.load(filename).flatten()
    print(f"Loaded sequence memory from {filename}")
    return data


def load_sequence_memory_extended(data_dir):
    """
    Loads the latest u_sm_1 and u_sm_2 sequence memory fields from the data directory
    using the provided get_latest_file function.
    """
    # Load u_sm_1
    u_sm_1_filepath = get_latest_file(data_dir, "u_sm_1_")
    if u_sm_1_filepath is None:
        raise IOError(f"No 'u_sm_1_' files found in the data directory: {data_dir}")
    
    u_sm_1_data = np.load(u_sm_1_filepath).flatten()
    rospy.loginfo(f"✓ Loaded u_sm_1 from {os.path.basename(u_sm_1_filepath)}")

    # Load u_sm_2
    u_sm_2_filepath = get_latest_file(data_dir, "u_sm_2_")
    if u_sm_2_filepath is None:
        raise IOError(f"No 'u_sm_2_' files found in the data directory: {data_dir}")
        
    u_sm_2_data = np.load(u_sm_2_filepath).flatten()
    rospy.loginfo(f"✓ Loaded u_sm_2 from {os.path.basename(u_sm_2_filepath)}")
    
    return u_sm_1_data, u_sm_2_data

def load_task_duration(data_dir, filename=None):
    if filename is None:
        files = [f for f in os.listdir(data_dir) if f.startswith("u_d_") and f.endswith('.npy')]
        if not files:
            raise IOError("No 'u_d_' files found in the data folder.")
        filename = max([os.path.join(data_dir, f) for f in files], key=os.path.getmtime)
    data = np.load(filename).flatten()
    print(f"Loaded task duration from {filename}, size: {data.size}, max: {data.max()}")
    return data

# Inside utils.py

def format_axis(ax, title, ylabel, xticks, xticklabels, vlines=None, ylim=(-5, 5), xlim=None):
    ax.set_title(title)
    ax.set_ylabel(ylabel)
 
    if xlim is not None:
        # If xlim is provided, use it.
        ax.set_xlim(*xlim)
    else:
        # Otherwise, fall back to the old behavior (fit to ticks).
        ax.set_xlim(min(xticks), max(xticks))

    ax.set_ylim(*ylim)
    ax.set_xticks(xticks)
    ax.set_xticklabels(xticklabels, rotation=45)
    ax.grid(True)
    if vlines is not None:
        for x in vlines:
            ax.axvline(x=x, color='gray', linestyle='--', alpha=0.3)
    return ax


def get_latest_file(data_dir, pattern):
    """Retrieve the latest file in the data directory matching the pattern."""
    files = [f for f in os.listdir(data_dir) if f.startswith(pattern) and f.endswith('.npy')]
    if not files:
        return None
    # Sort files by modified time (latest first)
    files.sort(key=lambda f: os.path.getmtime(os.path.join(data_dir, f)), reverse=True)
    return os.path.join(data_dir, files[0])


def save_field(data, prefix, data_dir="data"):
    """Save a single numpy array with timestamped filename."""
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(data_dir, f"{prefix}_{timestamp}.npy")
    np.save(filename, data)
    rospy.loginfo(f"Saved {prefix} to {filename}")
    return filename

def save_node_history(node, data_dir="data"):
    """Save all history fields of a node."""
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    rospy.loginfo(f"SAVING HISTORY to {data_dir}")
    history_fields = {
        "act_history": getattr(node, "u_act_history", None),
        "sim_history": getattr(node, "u_sim_history", None),
        "wm_history": getattr(node, "u_wm_history", None),
        "f1_history": getattr(node, "u_f1_history", None),
        "f2_history": getattr(node, "u_f2_history", None),
        "h_amem": getattr(node, "h_u_amem", None),
    }

    for name, data in history_fields.items():
        if data is not None:
            save_field(data, name, data_dir)