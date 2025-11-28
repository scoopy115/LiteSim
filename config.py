# config.py
import sys
import os

# --- APP INFO ---
APP_NAME = "LiteSim"
APP_VERSION = "1.9.7"
AUTHOR = "Dylan Kiesebrink"
GITHUB_URL = "https://github.com/scoopy115/LiteSim"
REPO_OWNER = "scoopy115"
REPO_NAME = "LiteSim"
PORTFOLIO_URL = "https://dylankiesebrink.nl"

# --- ROBOT CONFIGURATION ---
JOINT_COUNT = 6
ROBOT_Z_OFFSET = -0.19
SIM_SPEED_FACTOR = 1.0 
ROBOT_SCAN_PORT = 30002

# --- GLOBAL API REFERENCE ---
GLOBAL_API_INSTANCE = None 

# --- XARM SDK CHECK ---
try:
    from xarm.wrapper import XArmAPI as RealXArmAPI
    HAS_REAL_SDK = True
    print("[SYSTEM] xarm-python-sdk found! Working in simulation and robot mode.")
except ImportError:
    HAS_REAL_SDK = False
    print("[SYSTEM] xarm-python-sdk not found. Working in simulation-mode only.")

# --- FILE PATHS ---

if getattr(sys, 'frozen', False):
    PROJECT_ROOT = sys._MEIPASS
else:
    PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

USER_DATA_DIR = os.path.join(os.path.expanduser("~"), ".litesim")

if not os.path.exists(USER_DATA_DIR):
    try:
        os.makedirs(USER_DATA_DIR)
    except OSError:
        pass

# --- DEFINE PATHS ---

MODEL_DIR = os.path.join(PROJECT_ROOT, "model")
VISUAL_DIR = os.path.join(MODEL_DIR, "visual")
EXAMPLES_DIR = os.path.join(PROJECT_ROOT, "examples")
ICON_PATH = os.path.join(PROJECT_ROOT, "assets", "icon.png")
HISTORY_FILE = os.path.join(USER_DATA_DIR, "recent_scripts.txt")
STL_HISTORY_FILE = os.path.join(USER_DATA_DIR, "recent_stls.txt")

# --- WINDOW SETTINGS ---
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 1000
INITIAL_ZOOM_LEVEL = 2