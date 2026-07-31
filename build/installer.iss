; Inno Setup script for FH6 TC.
;
; Produces a single setup.exe that installs the app, creates shortcuts, and
; handles the two kernel-driver dependencies the app cannot work without.
;
; The dependencies are DOWNLOADED at install time from the vendor's official
; GitHub releases rather than bundled. That keeps this installer small, avoids
; redistributing someone else's signed driver packages, and means a customer
; always gets the vendor's own signed binary rather than a copy from us.
;
; Build:  "%LOCALAPPDATA%\Programs\Inno Setup 6\ISCC.exe" build\installer.iss
; Output: build\out\FH6-TC-Setup-<version>.exe
;
; Prerequisite: run PyInstaller first, so dist\FH6 TC\ exists.

#define AppName        "FH6 TC"
#define AppVersion     "1.0.0"
#define AppPublisher   "Talon Townsend"
#define AppExe         "FH6 TC.exe"
#define AppUrl         "https://github.com/talontownsend/fh6-traction-control"

#define ViGEmUrl  "https://github.com/nefarius/ViGEmBus/releases/download/v1.22.0/ViGEmBus_1.22.0_x64_x86_arm64.exe"
#define HidHideUrl "https://github.com/nefarius/HidHide/releases/download/v1.5.230.0/HidHide_1.5.230_x64.exe"

[Setup]
; A stable AppId is what lets future versions upgrade in place instead of
; installing alongside. Never change it once released.
AppId={{8F3A6C21-4B7E-4E19-9E4C-2D6B5A1F7C08}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher={#AppPublisher}
AppPublisherURL={#AppUrl}
AppSupportURL={#AppUrl}
DefaultDirName={autopf}\{#AppName}
DefaultGroupName={#AppName}
OutputDir=out
OutputBaseFilename=FH6-TC-Setup-{#AppVersion}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
; The app itself requires elevation (it drives HidHide), and it installs to
; Program Files, so ask once here rather than at every launch.
PrivilegesRequired=admin
ArchitecturesInstallIn64BitMode=x64compatible
ArchitecturesAllowed=x64compatible
; The EULA, NOT the repo's source license. MIT governs the source code and
; grants redistribution and resale rights, which is exactly wrong to show a
; paying customer as their terms of use.
LicenseFile=..\legal\EULA.txt
DisableProgramGroupPage=yes
UninstallDisplayIcon={app}\{#AppExe}
; Shown in the SmartScreen / UAC prompt, so make it recognisable.
VersionInfoCompany={#AppPublisher}
VersionInfoProductName={#AppName}
VersionInfoVersion={#AppVersion}

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Shortcuts:"

[Files]
; The whole PyInstaller --onedir output, including _internal.
Source: "..\dist\FH6 TC\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
; BSD-3 requires the ViGEmClient copyright notice and disclaimer to travel
; with binary redistributions, so these are a license obligation, not a nicety.
Source: "..\legal\THIRD-PARTY-NOTICES.txt"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\legal\EULA.txt"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExe}"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExe}"; Description: "Launch {#AppName}"; Flags: nowait postinstall skipifsilent

[Code]
var
  DepPage: TOutputProgressWizardPage;
  NeedViGEm, NeedHidHide: Boolean;

{ ViGEmBus registers a kernel service under this key. Presence of the service
  is a more reliable signal than a file path, which varies by version. }
function ViGEmInstalled(): Boolean;
begin
  Result := RegKeyExists(HKEY_LOCAL_MACHINE,
    'SYSTEM\CurrentControlSet\Services\ViGEmBus');
end;

{ HidHide ships its CLI to a fixed vendor path; the app shells out to exactly
  this executable, so checking for it tests what actually matters. }
function HidHideInstalled(): Boolean;
begin
  Result := FileExists(ExpandConstant('{commonpf}\Nefarius Software Solutions\HidHide\x64\HidHideCLI.exe'))
         or FileExists(ExpandConstant('{commonpf}\Nefarius Software Solutions\HidHide\HidHideCLI.exe'))
         or RegKeyExists(HKEY_LOCAL_MACHINE,
              'SYSTEM\CurrentControlSet\Services\HidHide');
end;

procedure InitializeWizard();
begin
  DepPage := CreateOutputProgressPage('Required drivers',
    'FH6 TC needs two small drivers to create its virtual controller and to ' +
    'hide your real one from the game.');
end;

function NextButtonClick(CurPageID: Integer): Boolean;
var
  Msg: String;
begin
  Result := True;
  if CurPageID = wpReady then
  begin
    NeedViGEm := not ViGEmInstalled();
    NeedHidHide := not HidHideInstalled();
    if NeedViGEm or NeedHidHide then
    begin
      Msg := 'FH6 TC needs the following, which are not installed yet:' #13#10;
      if NeedViGEm then
        Msg := Msg + #13#10 '  ViGEmBus  (creates the virtual controller. Required.)';
      if NeedHidHide then
        Msg := Msg + #13#10 '  HidHide  (hides your real controller from the game.' #13#10
                          + '            Without it the game reads both controllers' #13#10
                          + '            and the assists have no effect.)';
      Msg := Msg + #13#10#13#10 'Setup will download them from the vendor and run '
                 + 'their installers after copying FH6 TC.' #13#10#13#10
                 + 'Both may ask for permission and one may request a restart.' #13#10#13#10
                 + 'Continue?';
      Result := MsgBox(Msg, mbConfirmation, MB_YESNO) = IDYES;
    end;
  end;
end;

{ Download and run one dependency. Failure is reported but does not abort the
  install: a customer with the app installed and a driver missing can retry,
  whereas a rolled-back install leaves them with nothing. }
procedure InstallDependency(const Url, FileName, Title: String);
var
  Tmp: String;
  Code: Integer;
begin
  Tmp := ExpandConstant('{tmp}\') + FileName;
  DepPage.SetText('Downloading ' + Title + '...', '');
  try
    DownloadTemporaryFile(Url, FileName, '', nil);
  except
    MsgBox('Could not download ' + Title + '.' #13#10#13#10 +
           GetExceptionMessage + #13#10#13#10 +
           'FH6 TC is installed, but will not work until ' + Title +
           ' is present. You can install it yourself from:' #13#10 + Url,
           mbError, MB_OK);
    Exit;
  end;
  DepPage.SetText('Installing ' + Title + '...',
                  'Its own installer window may appear.');
  if not Exec(Tmp, '', '', SW_SHOW, ewWaitUntilTerminated, Code) then
    MsgBox('Could not start the ' + Title + ' installer.' #13#10 +
           'FH6 TC is installed, but will not work until ' + Title +
           ' is present.', mbError, MB_OK);
end;

procedure CurStepChanged(CurStep: TSetupStep);
begin
  if CurStep = ssPostInstall then
  begin
    if NeedViGEm or NeedHidHide then
    begin
      DepPage.Show;
      try
        if NeedViGEm then
          InstallDependency('{#ViGEmUrl}', 'ViGEmBus_Setup.exe', 'ViGEmBus');
        if NeedHidHide then
          InstallDependency('{#HidHideUrl}', 'HidHide_Setup.exe', 'HidHide');
      finally
        DepPage.Hide;
      end;
    end;
  end;
end;
