package de.trafficsimulation;

/**
An assortment of constants influencing the global appearance and functionality
of the applet. In many cases, the applet can be adapted to 
special needs (e.g. a different applet size, different time steps, different truck percentage etc) by simply changing some numbers here and recompiling.

 */
// if changes here, compile everything anow (rm *.class before compiling)
public interface Constants { // MicroApplet3_0

    // ###################################################
    // MicroApplet3_0: Most important overall properties
    // ###################################################
    static final int DEFAULT_LANG_INDEX = 1; //english=1; german=0
    static final int CHOICE_SCEN_INIT = 7;
    static final boolean DOUBLEBUF = true; // whether double-buffering used
    static final int FRAMES_PER_TIMESTEP = 1;   // simulation time step
    static final boolean STRAIGHT_ROAD = false;
    static final boolean CLOCKWISE = true; // car movement in ring geometry
    static final double TIMESTEP_S = 0.25;   // simulation time step
    //static final double FRAC_TRUCK_INIT = 0.2; // not for circle
    static final double FRAC_TRUCK_INIT = 0.1; // not for circle
    //static final double FRAC_TRUCK_INIT_CIRCLE = 0.2; 
    static final double FRAC_TRUCK_INIT_CIRCLE = 0.005;
    static final int CONTROL_SIZE = 2; // {relScreen, relBrowser, fix}
    static final double REL_APPL_SIZE = 0.70; // size / screen size
    static final int APPL_WIDTH = 1024;  //700 nicht relevant
    static final int APPL_HEIGHT = 760;   //550 fur ctrl_size!=2!
    static final boolean SHOW_INSET_DIAG = true;  // !! Fund diagrams etc!
    static final boolean SHOW_TEXT = false;
    // #######################################################
    // Lane-change parameters 
    // Safety:    BSAVE,SMIN
    // Incentive: BIAS*, P_FACTOR, DB
    // Law:       EUR_RULES,VC_MS
    // #######################################################
    static final boolean EUR_RULES = true;
    static final int VC_MS = 16;    // crit. velocity where Europ rules 
    // kick in (in m/s)
    static final double MAIN_BSAVE = 12.;
    static final double MAIN_BSAVE_SELF = 12.;
    static final double MAIN_SMIN = 2.;
    static final double RMP_BSAVE = 20.; // high to force merging
    static final double RMP_SMIN = 2.;  // min. safe distance
    static final double RMP_BIAS_LEFT = 8.;  // high to force merging (8)
    static final double LANECL_BSAVE = 12.; // high to force merging
    static final double LANECL_BIAS_RIGHT = 0.7; // negative to force merging
    // to the left
    static final double BIAS_RIGHT_CAR = 0.1; // right-lane bias: sesgo para carros entre los carriles  
    static final double BIAS_RIGHT_TRUCK = 0.3;
    static final double BIAS_RIGHT_CAR3 = 2; // right-lane bias : para el escenario 3
    static final double BIAS_RIGHT_TRUCK3 = 1;
    static final double P_FACTOR_CAR = 0.2;   // politeness factor
    static final double P_FACTOR_TRUCK = 0.2;
    static final double DB_CAR = 0.3;   // changing thresholds (m/s^2)
    static final double DB_TRUCK = 0.2;
    static final double DB_CAR3 = 0.3; // 0.3;  
    static final double DB_TRUCK3 = 0.2; // 0.2; //?!! aus irgendeinen Grund
    // etwa grob Faktor zehn bei ANgabe der db noetig ?!!??!!
    // ########################################################
    // interactive control variables (initial values in MicroSim*.java)
    // ########################################################
    //IDM control variables Martin jan05
    static final double V0_MIN_KMH = 1;
    static final double V0_MAX_KMH = 50;
    static final double V0_INIT_KMH = 35;
    static final double S0_MIN_M = 0;
    static final double S0_MAX_M = 6;
    static final double S0_INIT_M = 2;
    static final double S1_MIN_M = 0;
    static final double S1_MAX_M = 15;
    static final double S1_INIT_M = 5;
    static final double T_MIN_S = 0.3;
    static final double T_MAX_S = 3;
    //static final double T_INIT_S = 1.5;  
    static final double T_INIT_S = 1.5;
    static final double A_MIN_MSII = 0.3;
    static final double A_MAX_MSII = 3;
    static final double A_INIT_CAR_MSII = 0.5;
    static final double A_INIT_TRUCK_MSII = 0.4;
    static final double B_MIN_MSII = 0.5;
    static final double B_MAX_MSII = 5.0;
    static final double B_INIT_MSII = 3.0;
    static final double MAX_BRAKING = 20.0;
    // traffic control variables
    static final int DENS_MIN_INVKM = 4;   // (veh/km/ (2 lanes))
    static final int DENS_MAX_INVKM = 80;    // DENSIDAD MAXIMA DE VEHICULOS
    static final int DENS_INIT_INVKM = 40;    //
    static final int SPEED_MAX = 20;    // (ms per frame)
    static final int SPEED_MIN = 400;
    static final int SPEED_INIT = 25; // time warp!
    //static final double LNSPEED_MAX = Math.log(1./20);
    //static final double LNSPEED_MIN = Math.log(1./500); 
    static final int V0_LIMIT_MAX_KMH = 140;    // (km/h) (140: free)
    static final int V0_LIMIT_MIN_KMH = 20;
    static final int V0_LIMIT_INIT_KMH = 80;
    static final int VMAX_TRUCK_KMH = 80;
    static final int Q_MAX = 4000;     // (veh/h/ (2 lanes))
    ///////////////////////////////
    static final int QFM_MAX = 1000;
    static final int QFT_MAX = 1000;
    static final int QFV_MAX = 500;
    ///////////////////////////////
    
    static final int QRMP_MAX = 1800;       // (veh/h/lane)
    static final int Q_INIT7 = 800;  //3300
    static final int Q_INITFM = 488;// MERIDA-(TABAY-EL VALLE) HORA: 7:00am    
    //static final double PERCAR_FMTV = 0.8;//400*0.8=  320 CARROS 
    static final double PERTRUCK_FMTV = 0.135; // en la mañana
    //static final double PERCAR_FMTT = 0.8;//400*0.8=  320 CARROS 
    static final double PERTRUCK_FMTT = 0.196; // en la mañana
    // DE ESE 80% DE CARROS DESDE MERIDA,  
    //HACIA EL VALLE
    //static final double PERCAR_FM1 = 1-PERCAR_FM;// 320*0.2= 64 CARROS
    static final double PERCAR_FM1 = 0.285;// haia el valle
    static final double PERCAMIONES_FM1 = 0.02;
    static final double PERBUSES_FM1 = 0.;
    // HACIA TABAY
    //static final double PERCAR_FM2 = 1-PERCAR_FM1;// // 320*0.6=  192 CARROS
    static final double PERCAR_FM2=0.665;
    static final double PERCAMIONES_FM2 = 0.08;
    static final double PERBUSES_FM2 = 0.12;
    
    static final int Q_INITFT = 647;
    static final double PERCAR_FT = 0.8;
    static final double PERCAMIONES_FT = 0.08;
    static final double PERBUSES_FT = 0.12;
    
    static final int Q_INITFV = 201;
    static final double PERCAR_FV = 0.8;
    static final double PERCAMIONES_FV = 0.08;
    static final double PERBUSES_FV = 0.12;
    
    static final int QRMP_INIT2 = 400;
    static final int Q_INIT3 = 1600;
    static final int Q_INIT4 = 2800;
    static final int Q_INIT5 = 1800;
    //
    static final double SPACEMIN = 20.;
    
    static final int POLITENESS_MIN = -1;       // Politeness factor (0..P_MAX)
    static final int POLITENESS_MAX = 2;       // Politeness factor (0..P_MAX)
    static final double PRMP_MIN = 0;   // may be < 0
    static final double PRMP_MAX = 3;
    static final double DELTAB_MAX = 1;   // Switching threshold (m/s^2)  
    static final double DELTABRAMP_MIN = -2; // Switching threshold (m/s^2)  
    static final double DELTABRAMP_MAX = 1;   // Switching threshold (m/s^2)  
    static final double DELTABRAMP_INIT = -2;
    //static final double FRAC_TRUCK_INIT = 0.2; // oben!!
    //static final double FRAC_TRUCK_INIT_CIRCLE = 0.;   // oben!!
    // colors in MicroStreet.java (cars), SimCanvas.java
    // Layout windows
    static final int MARGIN = 1;  // space between windows and applet boundary
    static final int SB_SPACEX = 5;   // space between scrollbar window
    static final int SB_SPACEY = 2;   // elements
    // following only initial values => MicroSim.makeGlovbalLayout()
    static final int TEXTWINDOW_HEIGHT = 110;   // textarea with explanations
    static final int BUTTONWINDOW_HEIGHT = 50;
    static final int SBWINDOW_HEIGHT = 120;  // only init value
    static final int SB_IDM_HEIGHT = 150; // only init value  
    // Geometric simulation constants
    // onramp: Chose straight road geometry (true) or U-shaped (false)
    static final double STRAIGHT_RDLEN_M = 800.;
    static final double EXCESS_RDLEN_M = 40.;// original 100
    static final double STRAIGHT_RAMPPOS_M = 600.; // center of on-ramp
    static final double ANGLE_ACCESS = 0.4; // arctan(angle of access road)
    static final double ANGLE_ACCESSFM = 0.5;
    static final double ANGLE_ACCESSFVI = 0.2;// EN LA INTER
    static final double ANGLE_ACCESSFV = 0.025;
    static final double STRAIGHT_ASPECTRATIO = 0.75;// original 0.4
    //para el canal desde Merida HASTA LA INTERSECCION Y VICEVERSA
    
    static final double ANCHO_CRUCE = 20.;
    static final double LARGO_CRUCE = 10.;
    static final double CRUCE = 22.360679775; //(Math.sqrt(Math.pow(ANCHO_CRUCE,2)+Math.pow(LARGO_CRUCE,2)));
    static final double STRAIGHT_FROMMERIDA_M = 83.;
    static final double STRAIGHT_TOMERIDA_M = 83. + CRUCE ;
    //para el canal desde TABAY HASTA LA INTERSECCION Y VICEVERSA
    static final double STRAIGHT_FROMTOTABAY_M = 455.;
    // PARA EL CANAL DESDE EL VALLE HASTA LA INTERSECCION Y VICEVERSA
    static final double STRAIGHT_FROMTOVALLE_M = 700.;
    static final double STRAIGHT1_M = 455; // tres secciones rectas hasta el Rio Mucujun, via Tabay
    static final double STRAIGHT2_M = 173;
    static final double STRAIGHT3_M = 463;
    static final double INTERTT = CRUCE;
    
    //procedencia del vehiculo
    static final int ORIGINFM = 1;
    static final int ORIGINFT = 2;
    static final int ORIGINFV = 3;
    
    static final int PRIORFT = 4;// con prioridad =0
    static final int PRIORFM = 2; // con prioridad =1
    static final int PRIOR_MAX = 50;
    
    
    // U-shaped geometry : GEOMETRIA EN FORMA DE U
    static final double RADIUS_M = 20.;  // of circular road sections
    static final double L_RAMP_M = 100.;  // length of on ramp
    static final double L_STRAIGHT_M = 200.; // of straight sections of U 
    static final double REL_ROAD_MARGIN = 0.01; // relative space between roads
    static final double LANEWIDTH_M = 10.;  // width of one lane
    static final double LINELENGTH_M = 4.;  // white middle lines
    static final double GAPLENGTH_M = 10.;  // white middle lines
    static final double RELPOS_TL = 0.7;  // pos of traffic light/roadlength
    static final double RELPOS_LANECL = 0.7;
    static final double LANECL_LENGTH = 10;   // Length (m) of closed lane
    static final double RELPOS_SPEEDSIGN = 0.1;  // pos of speed-limit sign
    static final double VEH_WIDTH_M = 4.;  // of both cars and trucks
    static final double PKW_LENGTH_M = 6.; //Tamaño de carros
    static final double LKW_LENGTH_M = 8.; //Tamaño de BUSES
    static final double TAM_CAMION_CARGA = 10.;
    // Layout text ACHTUNG: compileRun.sh anwenden, sonst f... DOS am Werk!
    // applied in  ../../MicroSim.java (scrollbar text, buttons) and 
    // SimCanvas.java (text ("Time", "Car", "Truck") in simulation apart from scrollbars)
    static final double REL_TEXTHEIGHT = 0.026; // textheight/simWindow height
    static final double SBTEXT_MAINTEXT_HEIGHTRATIO = 0.8;
    static final int MAX_TEXTHEIGHT = 30;
    static final int MIN_TEXTHEIGHT = 6;
    // times
    static final double T_GREEN = 40.0; // traffic light turns green
    static final double TimeFT = 120.0;
    //static final double TIMESTEP_S=0.4;  // oben!!
    static final double INIT_SIMTIME_MS = 20;
    static final double NT_LINES = 5;    // redraw road lines every .. timesteps
    static final double NT_ROAD = 10;     // redraw road every .. timesteps
    // tsleepFrame_ms = max((tsleep_ms- SIMTIME_MS)/framesPerStep
    // - DRAWTIME_MS,0)
    // Umlaute
    static final char SHARP_S = '\337';
    //static final char SHARP_S = '\u00DF';
    static final char aeUML = '\u00E4';
    static final char AEUML = '\u00C4';
    static final char oeUML = '\u00F6';
    static final char OEUML = '\u00D4';
    static final char ueUML = '\u00FC';
    static final char UEUML = '\u00DC';
}
