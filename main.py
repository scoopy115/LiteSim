import os
import sys
import tkinter as tk
from tkinter import ttk
import threading
import time

# Crash prevention
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE" 

import config

class SplashScreen(tk.Tk):
    def __init__(self):
        super().__init__()
        
        self.overrideredirect(True)
        
        w, h = 500, 500
        ws = self.winfo_screenwidth()
        hs = self.winfo_screenheight()
        x = (ws/2) - (w/2)
        y = (hs/2) - (h/2)
        self.geometry('%dx%d+%d+%d' % (w, h, x, y))
        
        bg_color = config.COLOR_BG
        fg_color = config.COLOR_BASE
        accent_color = config.COLOR_PATH
        self.configure(background=bg_color)

        # Splash UI
        
        self.logo_img = None
        try:
            if os.path.exists(config.ICON_PATH):
                src = tk.PhotoImage(file=config.ICON_PATH)
                if src.height() > 128:
                    self.logo_img = src.subsample(2, 2)
                else:
                    self.logo_img = src
                
                tk.Label(self, image=self.logo_img, bg=bg_color).pack(pady=(40, 20))
        except: pass

        tk.Label(self, text=f"{config.APP_NAME}", font=("Segoe UI", 24, "bold"), 
                 bg=bg_color, fg=fg_color).pack()
        
        tk.Label(self, text=f"v{config.APP_VERSION}", font=("Segoe UI", 10), 
                 bg=bg_color, fg="#888888").pack(pady=(0, 20))

        self.status_lbl = tk.Label(self, text="Initializing...", font=("Segoe UI", 9), 
                                   bg=bg_color, fg="#aaaaaa")
        self.status_lbl.pack(side=tk.BOTTOM, pady=(0, 10))

        style = ttk.Style()
        style.theme_use('clam')
        style.configure("Splash.Horizontal.TProgressbar", 
                        troughcolor="#333333", 
                        background=accent_color, 
                        bordercolor=bg_color, 
                        lightcolor=accent_color, 
                        darkcolor=accent_color)
        
        self.progress = ttk.Progressbar(self, style="Splash.Horizontal.TProgressbar", 
                                        orient="horizontal", length=400, mode="determinate")
        self.progress.pack(side=tk.BOTTOM, pady=(0, 20))

        self.app_class = None
        
        threading.Thread(target=self._load_heavy_modules, daemon=True).start()

    def _update_status(self, text, percent):
        self.status_lbl.config(text=text)
        self.progress['value'] = percent
        self.update_idletasks()

    def _load_heavy_modules(self):
        try:
            # Base imports
            time.sleep(1.0) # Short delay for UX
            self.after(0, lambda: self._update_status("Loading 3D Engine...", 20))
            
            import pyvista 
            
            # Math libraries
            self.after(0, lambda: self._update_status("Loading Math Kernel...", 40))
            import numpy
            import scipy
            
            # IKPY
            self.after(0, lambda: self._update_status("Initializing Kinematics Solver...", 60))
            from ikpy.chain import Chain

            self.after(0, lambda: self._update_status("Loading Robot Drivers (xArm SDK)...", 70))
            try:
                import xarm.wrapper
            except ImportError:
                pass

            time.sleep(1.0) # Short delay for UX
            
            # GUI
            self.after(0, lambda: self._update_status("Building User Interface...", 80))
            from gui import ControlPanel
            
            self.app_class = ControlPanel
            
            self.after(0, lambda: self._update_status("Ready!", 100))
            time.sleep(1.0)
            
            self.after(0, self._launch_app)
            
        except Exception as e:
            self.after(0, lambda: self._update_status(f"Error: {e}", 0))
            print(f"Splash Error: {e}")

    def _launch_app(self):
        self.destroy()
        
        if self.app_class:
            app = self.app_class()
            app.mainloop()

if __name__ == "__main__":
    splash = SplashScreen()
    splash.mainloop()