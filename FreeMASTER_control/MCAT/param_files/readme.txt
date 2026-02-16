M1_params.txt is a default MCAT parameters file. MCAT automatically loads M1_param.txt file.
M1_params_linix.txt is a MCAT parameters file tuned for LINIX 45ZWN24-40 motor of the MTRDEVKSPNK144 kit.
M1_params_Buhler.txt is a MCAT parameters file tuned for Buhler motor 1.25.037.403.00.
M1_params_Sunrise95.txt is a MCAT parameters file tuned for Sunrise motor (95 W) of the MCSPTE1AK344.
To load appropriate parameters of selected motor (Sunrise motor, Buhler or LINIX) replace content of M1_param.txt with content of the file of selected motor (M1_params_Sunrise95.txt, M1_param_Buhler.txt or M1_params_linix.txt)
Generate configuration file in MCAT (PMSM_appconfig.h), precompile the the project and flash the target.



Motor wires order for the Sunrise motor (42BLY3A78-24110):
DEVKIT-MOTORGD Phase A terminal: YELLOW
DEVKIT-MOTORGD Phase B terminal: GREEN
DEVKIT-MOTORGD Phase C terminal: BLUE



Motor wires order for the Linix motor (45ZWN24-40):
DEVKIT-MOTORGD Phase A terminal: White
DEVKIT-MOTORGD Phase B terminal: Blue
DEVKIT-MOTORGD Phase C terminal: Green



Motor wires order for the Buhler motor (1.25.037.403.00):
DEVKIT-MOTORGD Phase A terminal: Black
DEVKIT-MOTORGD Phase B terminal: White
DEVKIT-MOTORGD Phase C terminal: Red