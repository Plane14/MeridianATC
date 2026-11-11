#ifndef AppVersion
  #error AppVersion must be supplied by the build script
#endif
#ifndef NumericVersion
  #error NumericVersion must be supplied by the build script
#endif
#ifndef EngineDir
  #error EngineDir must name the PyInstaller dist folder
#endif
#ifndef ShellDir
  #error ShellDir must name the published WPF shell folder
#endif
#ifndef OutputDir
  #error OutputDir must be supplied by the build script
#endif

#define AppName "MeridianATC"
#define AppPublisher "MeridianATC Contributors"
#define AppExeName "MeridianATC.exe"

[Setup]
AppId={{7E1B0C7A-2F4D-4E6B-9C1A-5B3D8E2F7A41}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher={#AppPublisher}
VersionInfoVersion={#NumericVersion}
VersionInfoCompany={#AppPublisher}
VersionInfoDescription=Flight simulation ATC companion (entertainment use only)
VersionInfoProductName={#AppName}
VersionInfoCopyright=Copyright (c) MeridianATC Contributors
DefaultDirName={localappdata}\Programs\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0.17763
OutputDir={#OutputDir}
OutputBaseFilename=MeridianATC-Setup-v{#AppVersion}-win-x64
LicenseFile=..\LICENSE
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
CloseApplications=yes
RestartApplications=no
SetupLogging=yes
UsePreviousAppDir=yes
Uninstallable=yes
UninstallDisplayIcon={app}\{#AppExeName}
ChangesAssociations=no
ChangesEnvironment=no

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
; The engine: the PyInstaller bundle in its own folder the shell launches.
Source: "{#EngineDir}\*"; DestDir: "{app}\engine"; \
    Flags: ignoreversion recursesubdirs createallsubdirs
; The WPF shell: the executable at the top of {app}.
Source: "{#ShellDir}\*"; DestDir: "{app}"; \
    Flags: ignoreversion recursesubdirs createallsubdirs
; The prompt set the renderer and the intent parser read.
Source: "..\prompts\*"; DestDir: "{app}\prompts"; \
    Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\MeridianATC"; Filename: "{app}\{#AppExeName}"; \
    WorkingDir: "{app}"
Name: "{autodesktop}\MeridianATC"; Filename: "{app}\{#AppExeName}"; \
    WorkingDir: "{app}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExeName}"; \
    Description: "{cm:LaunchProgram,MeridianATC}"; \
    Flags: nowait postinstall skipifsilent
