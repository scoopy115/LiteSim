import PyInstaller.__main__
import os
import sys
import shutil
import config

APP_NAME = config.APP_NAME
VERSION = config.APP_VERSION
AUTHOR = config.AUTHOR
ICON_FILE = "icon.icns" if sys.platform == "darwin" else "icon.ico"
ICON_PATH = os.path.join("assets", ICON_FILE)

SEPARATOR = ":" if sys.platform == "darwin" else ";"

# Windows
def create_windows_version_file():
    v_parts = VERSION.split(".")
    while len(v_parts) < 4: v_parts.append("0")
    version_tuple = ", ".join(v_parts)

    content = f"""
VSVersionInfo(
  ffi=FixedFileInfo(
    filevers=({version_tuple}),
    prodvers=({version_tuple}),
    mask=0x3f,
    flags=0x0,
    OS=0x40004,
    fileType=0x1,
    subtype=0x0,
    date=(0, 0)
    ),
  kids=[
    StringFileInfo(
      [
      StringTable(
        u'040904B0',
        [StringStruct(u'CompanyName', u'{AUTHOR}'),
        StringStruct(u'FileDescription', u'{APP_NAME} Control Panel'),
        StringStruct(u'FileVersion', u'{VERSION}'),
        StringStruct(u'InternalName', u'{APP_NAME}'),
        StringStruct(u'LegalCopyright', u'© {AUTHOR}. All rights reserved.'),
        StringStruct(u'OriginalFilename', u'{APP_NAME}.exe'),
        StringStruct(u'ProductName', u'{APP_NAME}'),
        StringStruct(u'ProductVersion', u'{VERSION}')])
      ]), 
    VarFileInfo([VarStruct(u'Translation', [1033, 1200])])
  ]
)
"""
    with open("version_info.txt", "w", encoding="utf-8") as f:
        f.write(content)
    return "version_info.txt"

# BUILD ARGS
args = [
    'main.py',                        
    f'--name={APP_NAME}',             
    '--noconsole',      
    '--onefile',                    
    '--clean',                        
    f'--icon={ICON_PATH}',            
    
    f'--add-data=assets{SEPARATOR}assets',
    f'--add-data=model{SEPARATOR}model',
    f'--add-data=examples{SEPARATOR}examples',
    
    '--hidden-import=PIL._tkinter_finder',
    '--hidden-import=vtkmodules',
    '--hidden-import=sv_ttk',
    '--hidden-import=robot_api',
    '--hidden-import=visualizer',
    '--hidden-import=config',
    '--hidden-import=utils',
]

if sys.platform == "win32":
    print("--- BUILDING FOR WINDOWS ---")
    v_file = create_windows_version_file()
    args.append(f'--version-file={v_file}')

elif sys.platform == "darwin":
    print("--- BUILDING FOR MACOS ---")
    bundle_id = f"nl.{AUTHOR.lower().replace(' ', '')}.{APP_NAME.lower()}"
    args.append(f'--osx-bundle-identifier={bundle_id}')

# PYINSTALLER
print(f"Start build v{VERSION} by {AUTHOR}...")
PyInstaller.__main__.run(args)

# MACOS ARGS
if sys.platform == "darwin":
    import plistlib
    
    app_path = os.path.join("dist", f"{APP_NAME}.app")
    plist_path = os.path.join(app_path, "Contents", "Info.plist")
    
    if os.path.exists(plist_path):
        print("Updating macOS Info.plist metadata...")
        with open(plist_path, 'rb') as f:
            pl = plistlib.load(f)
        
        # Inject metadata
        pl['CFBundleShortVersionString'] = VERSION
        pl['CFBundleVersion'] = VERSION
        pl['NSHumanReadableCopyright'] = f"© 2025 {AUTHOR}"
        pl['CFBundleGetInfoString'] = f"{APP_NAME} {VERSION}"
        
        with open(plist_path, 'wb') as f:
            plistlib.dump(pl, f)

# Cleanup
if os.path.exists("version_info.txt"):
    os.remove("version_info.txt")

print(f"\nBuild complete in /dist folder.")