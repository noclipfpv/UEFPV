UEFPV - FPV drone plugin for UEVR
=================================

FRANCAIS
1. Telechargez UEVR 1.05 (version stable) : https://github.com/praydog/UEVR/releases
2. Injectez le jeu une premiere fois avec UEVR, puis fermez le jeu.
3. Copiez uevr_fpv_drone.dll dans :
     %APPDATA%\UnrealVRMod\<NomDuJeu>\plugins\
   (ou %APPDATA%\UnrealVRMod\UEVR\plugins\ pour tous les jeux)
   Fichier bloque par Windows : clic droit > Proprietes > Debloquer.
4. Relancez le jeu et injectez avec UEVR.
   F6 = menu, F7 = drone, F8 = respawn, F10 = seconde fenetre.
5. Radio RC : branchee en mode joystick USB AVANT de lancer le jeu,
   F6 > Input > RC radio / joystick, Calibrate, Save (reglages par jeu).
   Radio absente : Steam > Proprietes du jeu > Manette > Desactiver Steam Input.
Bug : ouvrez une Issue sur GitHub avec log.txt et fpv_drone.ini
(dossier %APPDATA%\UnrealVRMod\<NomDuJeu>\).

ENGLISH
1. Get UEVR 1.05 (stable release): https://github.com/praydog/UEVR/releases
2. Inject the game once with UEVR, then close the game.
3. Copy uevr_fpv_drone.dll to:
     %APPDATA%\UnrealVRMod\<GameExeName>\plugins\
   (or %APPDATA%\UnrealVRMod\UEVR\plugins\ for every game)
   If Windows blocks it: right-click > Properties > Unblock.
4. Start the game and inject with UEVR.
   F6 = menu, F7 = drone, F8 = respawn, F10 = second window.
5. RC radio: plug it in USB joystick mode BEFORE starting the game,
   F6 > Input > RC radio / joystick, Calibrate, Save (settings are per game).
   Radio missing: Steam > game Properties > Controller > Disable Steam Input.
Bugs: open a GitHub Issue with log.txt and fpv_drone.ini
(from %APPDATA%\UnrealVRMod\<GameExeName>\).

Built for UEVR 1.05. UEVR by praydog (MIT). This plugin: MIT, see LICENSE.
