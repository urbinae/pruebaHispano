
import java.applet.Applet;
import java.awt.Button;
import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Event;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.Label;
import java.awt.Panel;
import java.awt.Scrollbar;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;

import de.trafficsimulation.AdjustmentMethods;
import de.trafficsimulation.Constants;
import de.trafficsimulation.IDM;
import de.trafficsimulation.IDMCar;
import de.trafficsimulation.IDMTruck;
import de.trafficsimulation.Language;
import de.trafficsimulation.SimCanvas;
import de.trafficsimulation.TextCanvas1;

public class MicroSim extends Applet implements Constants {

    private static final long serialVersionUID = 1L;
    //Eleccion de escenario
    private int choice_Scen;
    private int tsleep_ms = SPEED_INIT;
    private double qIn = (CHOICE_SCEN_INIT == 7)
            ? Q_INIT7 / 3600. : (CHOICE_SCEN_INIT == 3)
            ? Q_INIT3 / 3600. : (CHOICE_SCEN_INIT == 4)
            ? Q_INIT4 / 3600. : Q_INIT5 / 3600.;
    private double qInFM = Q_INITFM / 3600.;
    private double qInFM1 = qInFM * PERCAR_FM1;
    private double qInFM2 = qInFM * PERCAR_FM2;
    private double qInFT = Q_INITFT / 3600.;
    private double qInFV = Q_INITFV / 3600.;
    // Atributos para el canal desde merida
    private double v0 = 40 / 3.6;// velocidad inicial
    private double perTrFM = 0.12;// porcentaje de camiones
    private double perTrFM1 = 0.02;// = perTrFM - perTrFM2
    private double perTrFM2 = 0.06;
    private double perTrFT = 0.15;
    private double perTrFV = 0.124;
    private int priorFM = PRIORFM;
    private int priorFT = PRIORFT;
// factor de cambio de canal en la rampa
    private double p_factorRamp = 0.; // ramp Lanechange factor
// factor de cambio de canal en la rampa, 
// static final double DELTABRAMP_INIT = -2 
    private double deltaBRamp = DELTABRAMP_INIT; // ramp Lanechange factor
// limite de velocidad
    private double v0_limit = (CHOICE_SCEN_INIT == 3) ? 80 / 3.6 : 120 / 3.6; // speed limit (m/s)
// static final double FRAC_TRUCK_INIT_CIRCLE = 0.005
// Fraccion inicial de camiones en circulo
    private double perTr = FRAC_TRUCK_INIT_CIRCLE; // truck fraction !!!
    private double p_factor = -0.99; // lanechanging: politeness factor : factor de cortesia
    private double deltaB = 0; // lanechanging: changing threshold : umbral de desaceleracion cambio
//umbral: Cantidad mínima necesaria de una magnitud para el inicio de una acción
    // martin jan 05
    private IDM idmCar; //=new IDMCar(); // default params in constructor call
    private IDM idmTruck; //=new IDMTruck(); // default params in constructor call
    // Colors
    final Color BACKGROUND_COLOR = new Color((float) (0.8), (float) (0.8), (float) (0.8));// gris
    final Color SIM_BG_COLOR = new Color(0, 150, 0); // green grass
    final Color BUTTON_COLOR = new Color((float) (0.8), (float) (0.8), (float) (0.8));// gris
    // text parameters, labels: // parametros de texto, etiquetas
    // (if same labels in more than 1 scenario,
    // postfixes give number of relevant scenarios where 3 stands for 3-5)
    private Label label_simSpeed1, label_simSpeed2, // etiqueta para velocidad1, velocidad2, velocidad3 y velocidad6
            label_simSpeed3, label_simSpeed6;
    private Label label_qIn2, label_qIn3; // flujo de entrada 2 y 3
    private Label label_p_factorRamp; // para el factor p de la rampa
    private Label label_deltaBRamp; // para el deltaB de la rampa
    private Label label_perTr1; // porcentaje de camiones 1
    private Label label_perTr35; // porcentaje de camiones 35
    private Label label_v0_limit; // para el limite de la velocidad inicial
    private Label label_p_factor2, label_p_factor35, label_p_factor6; //noetig!!
    private Label label_deltaB2, label_deltaB35, label_deltaB6;
    // Etiquetas para la interseccion
    private Label label_qInFM;
    private Label label_qInFM1;
    private Label label_qInFM2;
    private Label label_qInFT1;
    private Label label_qInFV1;
    private Label label_qInFT2;
    private Label label_qInFV2;
    private Label label_priorityFM1;
    private Label label_priorityFT1;
    private Label label_priorityFM2;
    private Label label_priorityFT2;
    // String.valueOf: paratransformar valores de otros tipos de datos a su 					// representación como cadena
// para la velocidad inicial del modelo del conductor inteligente
// static final double V0_INIT_KMH = 120
    private Label label_IDMv0 = new Label(String.valueOf(V0_INIT_KMH) + " km/h");
    //private Label label_IDMv0
    // = new Label("TestTestTestTest");
    // Brecha de tiempo deseada
//static final double T_INIT_S = 1.5
    private Label label_IDMT = new Label(String.valueOf(T_INIT_S) + " s");
// aceleracion
// static final double A_INIT_CAR_MSII = 0.5
    private Label label_IDMa = new Label(String.valueOf(A_INIT_CAR_MSII) + " m/s^2");
// desaceleracion
//static final double B_INIT_MSII = 3.0
    private Label label_IDMb = new Label(String.valueOf(B_INIT_MSII) + " m/s^2");
// brecha minima entre los vehiculos con velocidad cero
//static final double S0_INIT_M = 2 metros
    private Label label_IDMs0 = new Label(String.valueOf(S0_INIT_M) + " m");
// brecha deseada
//static final double S1_INIT_M = 5 metros
    private Label label_IDMs1 = new Label(String.valueOf(S1_INIT_M) + " m");
// valores del modelo del conductor inteligente de tipo cadena
    private String str_IDMv0;
    private String str_IDMT;
    private String str_IDMa;
    private String str_IDMb;
    private String str_IDMs0;
    private String str_IDMs1;
    private String str_avgdens; // densidad promedio
    private String str_inflow; // flujo de entrada
    private String str_rmpinflow; // flujo de entrada en la rampa
    private String str_polite; // cortesia
    private String str_rmppolite; // cortesia de la rampa
    private String str_trucks;// camiones
    private String str_db; // agresividad o timidez al conducir en la via principal
    private String str_rmpdb; // agresividad o timidez al conducir en la rampa
    private String str_speed;// rapidez de la simulacion
    private String str_framerate; //velocidad de cuadro
    private String str_speedl; // velocidad l
    private String str_vehperh; //vehiculos por hora
    private String str_vehperkm; // vehiculos por kilometros
    private String str_veh; // vehiculos 
    private String str_button1; // botones para los escenarios
    private String str_button2;
    private String str_button3;
    private String str_button4;
    private String str_button5;
    private String str_button6; // boton para el vehiculo de perturbacion
    // cadenas para los flujos de entrada
    private String str_qInFM;
    private String str_qInFM1;
    private String str_qInFM2;
    private String str_qInFT;
    private String str_qInFV;
    private String str_priorityFM;
    private String str_priorityFT;
    // Cadenas para las colas
    //private String str_colaFM;
    private String str_colaFT;
    private String str_colaFV;
    private String str_colaTM;
    private String str_colaTT;
    private String str_colaTV;
    private String str_cola;
    // interactive control (1:1 to corresponding labels)
// control interactivo de algunas variables
    private Scrollbar sb_density; // densidad 
    private Scrollbar sb_simSpeed1, sb_simSpeed2, sb_simSpeed3, sb_simSpeed6; // velocidad 1, 2, 3 y 6
    private Scrollbar sb_qIn2, sb_qIn3; // flujo de entrada 2 y 3
    private Scrollbar sb_qRamp; // flujo de la rampa
    private Scrollbar sb_p_factorRamp; // factor de amabilidad o cortesia
    private Scrollbar sb_deltaBRamp; // umbral de cambio (desaceleracion)
    private Scrollbar sb_perTr1; // porcentaje de camiones
    private Scrollbar sb_perTr35;
    private Scrollbar sb_v0_limit; // limite de velocidad dada, inicial
    private Scrollbar sb_p_factor2, sb_p_factor35, sb_p_factor6; // para los escenarios del 2 al 6
    private Scrollbar sb_deltaB2, sb_deltaB35, sb_deltaB6;
    //scrollbar para la interseccion
    private Scrollbar sb_qInFM;
    private Scrollbar sb_qInFM1;
    private Scrollbar sb_qInFM2;
    private Scrollbar sb_qInFT1;
    private Scrollbar sb_qInFT2;
    private Scrollbar sb_qInFV1;
    private Scrollbar sb_qInFV2;
    private Scrollbar sb_priorityFM1;
    private Scrollbar sb_priorityFT1;
    private Scrollbar sb_priorityFM2;
    private Scrollbar sb_priorityFT2;
    // scrollbar
    // para los parametros del modelo
    private Scrollbar sb_IDMv0 = AdjustmentMethods.getSB(V0_MIN_KMH, V0_MAX_KMH, V0_INIT_KMH, false);
    private Scrollbar sb_IDMT = AdjustmentMethods.getSB(T_MIN_S, T_MAX_S, T_INIT_S, false);
    private Scrollbar sb_IDMa = AdjustmentMethods.getSB(A_MIN_MSII, A_MAX_MSII, A_INIT_CAR_MSII, false);
    private Scrollbar sb_IDMb = AdjustmentMethods.getSB(B_MIN_MSII, B_MAX_MSII, B_INIT_MSII, false);
    private Scrollbar sb_IDMs0 = AdjustmentMethods.getSB(S0_MIN_M, S0_MAX_M, S0_INIT_M, false);
    private Scrollbar sb_IDMs1 = AdjustmentMethods.getSB(S1_MIN_M, S1_MAX_M, S1_INIT_M, false);
    // buttons
    private Button start_button; // Start
    private Button stop_button; // Stop
    private Button button1; // interseccion actual
    private Button button2; // Ampliacion To Merida
    private Button button3; // Ampliacion From Merida
    private Button button4; // Ampliacion de From Merida y To Tabay
    private Button button5; // Traffic Lights
    private Button button6; // Aplly Perturbation
    // graphical components
    private int textHeight; // in pixels
    private Panel pClosedSystem = new Panel(); // scrollbars for closed system, Szen = 1
    private Panel pRamp = new Panel(); // scrollbars Ramp szenario 2
    private Panel pSource = new Panel(); // scenarios 3-5
    private Panel pLanechange = new Panel(); // Slalom Szen. 6
    private Panel pIDMparams = new Panel(); // extra scrollbars for IDM params
    private Panel pButtons = new Panel(); // scenario button field
    private Panel pScrollbars = new Panel(); // scrollbar field
    // Paneles para los flujos de entrada: From Merida, from Tabay and from Valle    
    
    private Panel pInFlow1 = new Panel(); // escenarios 7 y 8  
    
    private Panel pInFlow2 = new Panel(); // escenarios 9 y 10
    private CardLayout cardLayout = new CardLayout(0, 10); // for pScrollbars
    public static SimCanvas simCanvas; //=new SimCanvas (2, density, qIn, perTr, p_factor, deltaB, p_factorRamp, deltaBRamp, tsleep_ms);// simulation field
   
    private TextCanvas1 textCanvas = new TextCanvas1(1); // brown text field
    private Language lang;

//    public static void main(String[] args){
//        
//    }
    public void init() { // inicio del simulador

        lang = Language.getInstance(); // retorna el lenguaje por defecto, vacio

        // start with default language as defined in Constants
        //lang.setIndex(DEFAULT_LANG_INDEX);
        String userLangIndexStr = this.getParameter("language");
        if (userLangIndexStr != null) {
            int userLangIndex = Integer.parseInt(userLangIndexStr);
            lang.setIndex(userLangIndex);
            System.out.println("set language to index=" + lang.index());
        }
        setLanguage();

        // set default scenario
        choice_Scen = 7;
        String userScenarioStr = this.getParameter("scenario");
        if (userScenarioStr != null) {
            System.out.println("userScenario = " + userScenarioStr);
            int userScenario = Integer.parseInt(userScenarioStr);
            switch (userScenario) {
                case 1:
                    choice_Scen = 1;
                    break; // ringroad: 
                case 2:
                    choice_Scen = 2;
                    break; // onramp: rampa
                case 3:
                    choice_Scen = 3;
                    break; // laneclosing: canal cerrado
                case 4:
                    choice_Scen = 4;
                    break; // uphill: cuesta arriba
                case 5:
                    choice_Scen = 5;
                    break; // traffic lights: semaforo
                case 7:
                    choice_Scen = 7;
                    break;// interseccion
                default:
                    System.out.println("parameter: scenario index " + userScenario + " is not implemented ... ");
            }
        }
        System.out.println("choose simulation scenario =" + choice_Scen);

        // init.
        //idmCar = new IDMCar();
        //idmTruck = new IDMTruck();
        //simCanvas = new SimCanvas(choice_Scen, 0, qInFM, qInFM1, qInFM2, perTr, p_factor, deltaB, p_factorRamp, deltaBRamp, tsleep_ms);// simulation field
        //simCanvas = new SimCanvas(choice_Scen, priorFM, priorFT, qInFM, qInFM1, qInFM2, perTrFM, perTrFT, perTrFV, p_factor, deltaB, p_factorRamp, deltaBRamp, tsleep_ms);// simulation field

        simCanvas = new SimCanvas(choice_Scen, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, p_factor, deltaB, tsleep_ms);// simulation field

// determine size of screen, application, and client area
// usedWidt in makeGlobalLayout

        // size given relative to screen

        Dimension screensize = getToolkit().getScreenSize();
        if (CONTROL_SIZE == 0) {

            setSize((int) (REL_APPL_SIZE * screensize.width), (int) (REL_APPL_SIZE * screensize.height));
        } // size given by applet tag => by size of browser window
        else if (CONTROL_SIZE == 1) {;
        } // fixed size
        else if (CONTROL_SIZE == 2) {
            setSize(APPL_WIDTH, APPL_HEIGHT);
        }

        setFonts();
        setBackground(BACKGROUND_COLOR);



// objetos de la clase Panel
        pButtons.setLayout(new GridLayout(2, 3)); // 2 filas y 4 columnas para los botones de los distintos escenarios
        pInFlow1.setLayout(new GridBagLayout()); // Contenedor fuente 
        pRamp.setLayout(new GridBagLayout()); // otro contenedor para la rampa de acceso
        pLanechange.setLayout(new GridBagLayout()); // otro contenedor para el cambio de canal
        pClosedSystem.setLayout(new GridBagLayout()); // Contenedor para el sistema cerrado

        //pInFlow1.setLayout(new GridBagLayout());// contenedor para los flujos de entrada
        pInFlow2.setLayout(new GridBagLayout());
        //pResultados.setLayout(new GridBagLayout());// para los resultados
        //pColas.setLayout(new GridBagLayout());
        //pTiempos.setLayout(new GridBagLayout());


//martin jan05
        pIDMparams.setLayout(new GridBagLayout()); // Contenedor para el modelo IDM
        pIDMparams.setBackground(SIM_BG_COLOR); // as sim backgr, final Color SIM_BG_COLOR = new Color(0,150,0); // green grass
//pIDMparams.setBackground(Color.green); // brighter

//CardLayout cardLayout = new CardLayout(10,10) above;
// Only 1 component visible: select with
// cardLayout.show(component,string);
// string = same as in the cardLayout.add method

        pScrollbars.setLayout(cardLayout);
        pScrollbars.setBackground(Color.WHITE); // as sim backgr, Color.lightGray -> color de la via
        //panel2.setLayout(cardLayoutPanel);
        //panel2.setBackground(Color.WHITE);
        //pScrollbars.setBackground(new Color(100, 255, 50)); // as sim backgr
// pScrollbars.setBackground(Color.BUTTON_COLOR); // if outside

        /*pInFlow.setLayout(cardLayoutInFlow);// cardLayoutInFlow=CarLayout(0,3)
        pInFlow.setBackground(SIM_BG_COLOR); // as sim backgr
        pInFlow.setBackground(new Color(100,255,50)); // as sim backgr*/

        start_button = new Button("Iniciar");
        start_button.setForeground(Color.black);
        start_button.setBackground(BACKGROUND_COLOR);
        pButtons.add(start_button);

        button1 = new Button(str_button1);
        button1.setForeground(Color.black);
        button1.setBackground(BUTTON_COLOR);
        pButtons.add(button1);

        button3 = new Button(str_button3);
        button3.setForeground(Color.black);
        button3.setBackground(BUTTON_COLOR);
        pButtons.add(button3);

        /*button5 = new Button(str_button5);
        button5.setForeground(Color.black);
        button5.setBackground(BUTTON_COLOR);
        pButtons.add(button5);*/

        stop_button = new Button("Detener");
        stop_button.setForeground(Color.black);
        stop_button.setBackground(BUTTON_COLOR);
        pButtons.add(stop_button);

        button2 = new Button(str_button2);
        button2.setForeground(Color.black);
        button2.setBackground(BUTTON_COLOR);
        pButtons.add(button2);

        button4 = new Button(str_button4);
        button4.setForeground(Color.black);
        button4.setBackground(BUTTON_COLOR);
        pButtons.add(button4);

        /*button6 = new Button(str_button6);
        button6.setForeground(Color.red);
        button6.setBackground(BUTTON_COLOR);
        pButtons.add(button6);*/

// #######################################
// Define scrollbars
// #######################################

//        int idm_v0 = (int) (idmCar.v0);
     /*   int idm_a10 = (int) (idmCar.a * 10);
        int idm_b10 = (int) (idmCar.b * 10);
        int idm_T10 = (int) (idmCar.T * 10);
        int idm_s010 = (int) (idmCar.s0 * 10);*/


        int flow_invh = (int) (3600.0 * qIn);
        //Flujos de entrada
        int inFlowFM = (int) (3600.0 * qInFM);
        int inFlowFM1 = (int) (3600.0 * qInFM1);
        int inFlowFM2 = (int) (3600.0 * qInFM2);
        int inFlowFT = (int) (3600.0 * qInFT);
        int inFlowFV = (int) (3600.0 * qInFV);
        ////////////////////////////

        //Prioridades
        int priorFM = PRIORFM;
        int priorFT = PRIORFT;
        ////////////////////////////
        int lnspeed100init = (int) (100 * Math.log(1. / SPEED_INIT));
        int lnspeed100max = (int) (100 * Math.log(1. / SPEED_MIN)); //MIN !!
        int lnspeed100min = (int) (100 * Math.log(1. / SPEED_MAX));
        int truckPerc = (int) (100.0 * perTr);
        int truckPerc1 = (int) (100 * FRAC_TRUCK_INIT_CIRCLE);
        int p_factor100 = (int) (p_factor * 100);
        int p_factor100Ramp = (int) (p_factorRamp * 100);
        int deltaB100 = (int) (100.0 * deltaB);
        int deltaB100Ramp = (int) (100.0 * deltaBRamp);
        int i_warpfactor = (int) (1000 * TIMESTEP_S / tsleep_ms); //!! at initial
        String str_warpfactor = String.valueOf(i_warpfactor)
                + "."
                + String.valueOf((int) (10000 * TIMESTEP_S / tsleep_ms - 10 * i_warpfactor));



        sb_simSpeed1 = getSB(lnspeed100max, lnspeed100min, lnspeed100init);
        sb_simSpeed2 = getSB(lnspeed100max, lnspeed100min, lnspeed100init);
        sb_simSpeed3 = getSB(lnspeed100max, lnspeed100min, lnspeed100init);
        sb_simSpeed6 = getSB(lnspeed100max, lnspeed100min, lnspeed100init);
        sb_qIn2 = getSB(0, Q_MAX, flow_invh);
        ///////////////////////////////
        sb_qInFM = getSB(0, QFM_MAX, inFlowFM);
        sb_qInFM1 = getSB(0, QFM_MAX, inFlowFM1);
        sb_qInFM2 = getSB(0, QFM_MAX, inFlowFM2);
        sb_qInFT1 = getSB(0, QFT_MAX, inFlowFT);
        sb_qInFV1= getSB(0, QFV_MAX, inFlowFV);
        sb_qInFT2 = getSB(0, QFT_MAX, inFlowFT);
        sb_qInFV2 = getSB(0, QFV_MAX, inFlowFV);
        ///////////////////////////////////
        sb_priorityFM1 = getSB(0, PRIOR_MAX, priorFM);
        sb_priorityFT1 = getSB(0, PRIOR_MAX, priorFT);
        sb_priorityFM2 = getSB(0, PRIOR_MAX, priorFM);
        sb_priorityFT2 = getSB(0, PRIOR_MAX, priorFT);
        //////////////////////////////////
        sb_qIn3 = getSB(0, Q_MAX, flow_invh);
        sb_p_factorRamp = getSB((int) (100 * PRMP_MIN), (int) (100 * PRMP_MAX),
                p_factor100Ramp);
        sb_deltaBRamp = getSB((int) (100 * DELTABRAMP_MIN),
                (int) (100 * DELTABRAMP_MAX), deltaB100Ramp);
        sb_perTr1 = getSB(0, 100, (int) (100 * FRAC_TRUCK_INIT_CIRCLE));
        sb_perTr35 = getSB(0, 100, truckPerc);
//sb_v0_limit = getSB(0, 140, 80);
        sb_v0_limit = getSB(V0_LIMIT_MIN_KMH, V0_LIMIT_MAX_KMH,
                V0_LIMIT_INIT_KMH);
        sb_p_factor2 = getSB(100 * POLITENESS_MIN, 100 * POLITENESS_MAX, p_factor100);
        sb_p_factor35 = getSB(100 * POLITENESS_MIN, 100 * POLITENESS_MAX, p_factor100);
        sb_p_factor6 = getSB(100 * POLITENESS_MIN, 100 * POLITENESS_MAX, p_factor100);
        sb_deltaB2 = getSB(0, (int) (100 * DELTAB_MAX), deltaB100);
        sb_deltaB35 = getSB(0, (int) (100 * DELTAB_MAX), deltaB100);
        sb_deltaB6 = getSB(0, (int) (100 * DELTAB_MAX), deltaB100);

// #######################################
// Make Layout for scrollbars for 4 panels Szen 1,2,3-5,6
// #######################################

        GridBagConstraints gbconstr = new GridBagConstraints();

// 1th column: Variable names


        gbconstr.insets = new Insets(SB_SPACEY, SB_SPACEX, SB_SPACEY, SB_SPACEX);
        // (N,W,S,E)
        gbconstr.gridx = 0;
        gbconstr.gridy = 0;
        gbconstr.gridwidth = 1;
        gbconstr.gridheight = 1;
        gbconstr.fill = GridBagConstraints.NONE;
        gbconstr.anchor = GridBagConstraints.EAST;

        //Columna 1
        gbconstr.weightx = 0.;
        gbconstr.weighty = 1.;


        //pClosedSystem.add(new Label(str_avgdens), gbconstr);// escenario 1
        //pInFlow1.add(new Label(str_inflow), gbconstr); // scenarios 3-5        
        //pRamp.add(new Label(str_inflow), gbconstr); // Ramp szenario 2
        //pLanechange.add(new Label(str_polite), gbconstr); // szenario6
        ///////////////////////// Flujo de entrada From Merida
        //pInFlow1.add(new Label(str_qInFM), gbconstr);
        pInFlow1.add(new Label(str_qInFM), gbconstr);
        pInFlow2.add(new Label(str_qInFM1), gbconstr);
        //pColas.add(new Label(str_colaFM), gbconstr);
        /////////////////////////

        gbconstr.gridx = 0;
        gbconstr.gridy = 1;

        //pClosedSystem.add(new Label(str_trucks), gbconstr);
        //pRamp.add(new Label(str_rmpinflow), gbconstr);
        //pRamp.add(label_simSpeed2 = new Label(str_warpfactor + str_framerate), gbconstr);
        //pInFlow1.add(new Label(str_trucks), gbconstr);
        //pLanechange.add(new Label(str_db), gbconstr);
        ///////////////////////////// Flujo de entrada From Tabay
        //pInFlow1.add(new Label(str_qInFT), gbconstr);
        pInFlow1.add(new Label(str_qInFT), gbconstr);
        pInFlow2.add(new Label(str_qInFM2), gbconstr);
        //pColas.add(new Label("Colas"), gbconstr);
        //////////////////////////

        gbconstr.gridx = 0;
        gbconstr.gridy = 2;

        //pClosedSystem.add(new Label(str_speed), gbconstr);
        //pRamp.add(new Label(str_polite), gbconstr); //!! str_rmppolite
        //Label lab_speedl = new Label(str_speedl);
        //lab_speedl.setForeground(Color.red);
        //pInFlow1.add(lab_speedl, gbconstr);
        //pLanechange.add(new Label(str_speed), gbconstr);
        ///////////////////////////// Flujo de entrada From Valle
        //pInFlow1.add(new Label(str_qInFV), gbconstr);
        pInFlow1.add(new Label(str_qInFV), gbconstr);
        pInFlow2.add(new Label(str_qInFT), gbconstr);
        //pColas.add(new Label("Colas"), gbconstr);
        //////////////////////////

        gbconstr.gridx = 0;
        gbconstr.gridy = 3;

        //pRamp.add(new Label(str_db), gbconstr); //!! str_rmpdb
        //pInFlow1.add(new Label(str_polite), gbconstr);
        //pInFlow1.add(new Label(str_priorityFM), gbconstr);
        pInFlow1.add(new Label(str_priorityFM), gbconstr);
        pInFlow2.add(new Label(str_qInFV), gbconstr);
        //pColas.add(new Label("Colas"), gbconstr);


        gbconstr.gridx = 0;
        gbconstr.gridy = 4;

        //pRamp.add(new Label(str_speed), gbconstr);
        //pInFlow1.add(new Label(str_speed), gbconstr);
        //pInFlow1.add(new Label(str_priorityFT), gbconstr);
        pInFlow1.add(new Label(str_priorityFT), gbconstr);
        pInFlow2.add(new Label(str_priorityFM), gbconstr);
        //pInFlow2.add(new Label(str_speed), gbconstr);
        //pColas.add(new Label("Colas"), gbconstr);
        
        gbconstr.gridx = 0;
        gbconstr.gridy = 5;
        
        pInFlow1.add(new Label(str_speed), gbconstr);
        pInFlow2.add(new Label(str_priorityFT), gbconstr);
        
        gbconstr.gridx = 0;
        gbconstr.gridy = 6;
        pInFlow2.add(new Label(str_speed), gbconstr);

// 2th column: actual scrollbars

        gbconstr.gridx = 1;
        gbconstr.gridy = 0;
        gbconstr.weightx = 1.;
        gbconstr.fill = GridBagConstraints.HORIZONTAL;
        gbconstr.anchor = GridBagConstraints.CENTER;

        //pClosedSystem.add(sb_density, gbconstr);
        //pRamp.add(sb_qIn2, gbconstr);
        //pInFlow1.add(sb_qIn3, gbconstr);
        //pLanechange.add(sb_p_factor6, gbconstr);
        
        //pInFlow1.add(sb_qInFM, gbconstr);
        pInFlow1.add(sb_qInFM, gbconstr);
        pInFlow2.add(sb_qInFM1, gbconstr);
        //pColas.add(label_colaFM = new Label(String.valueOf(nvehFM) + str_cola), gbconstr);


        gbconstr.gridx = 1;
        gbconstr.gridy = 1;

        //pClosedSystem.add(sb_perTr1, gbconstr);
        //pRamp.add(sb_qRamp, gbconstr);
        //pInFlow1.add(sb_perTr35, gbconstr);
        //pLanechange.add(sb_deltaB6, gbconstr);

        //pInFlow1.add(sb_qInFV, gbconstr);
        pInFlow1.add(sb_qInFT1, gbconstr);
        pInFlow2.add(sb_qInFM2, gbconstr);

        gbconstr.gridx = 1;
        gbconstr.gridy = 2;

        //pClosedSystem.add(sb_simSpeed1, gbconstr);
        //pRamp.add(sb_p_factor2, gbconstr); // !! sb_p_factorRamp
        //pInFlow1.add(sb_v0_limit, gbconstr);
        //pLanechange.add(sb_simSpeed6, gbconstr);
        //pInFlow1.add(sb_qInFV, gbconstr);
        pInFlow1.add(sb_qInFV1, gbconstr);
        pInFlow2.add(sb_qInFT2, gbconstr);

        gbconstr.gridx = 1;
        gbconstr.gridy = 3;

        //pRamp.add(sb_deltaB2, gbconstr); //!! sb_deltaBRamp
        //pInFlow1.add(sb_p_factor35, gbconstr);
        //pInFlow1.add(sb_simSpeed2, gbconstr);
        pInFlow1.add(sb_priorityFM1, gbconstr);
        pInFlow2.add(sb_qInFV2, gbconstr);

        gbconstr.gridx = 1;
        gbconstr.gridy = 4;

        //pRamp.add(sb_simSpeed2, gbconstr);
        //pInFlow1.add(sb_simSpeed3, gbconstr);
        //pInFlow1.add(sb_simSpeed3, gbconstr);
        pInFlow1.add(sb_priorityFT1, gbconstr);
        pInFlow2.add(sb_priorityFM2, gbconstr);
        //pInFlow2.add(sb_simSpeed2, gbconstr);

        gbconstr.gridx = 1;
        gbconstr.gridy = 5;
        pInFlow1.add(sb_simSpeed3, gbconstr);
        pInFlow2.add(sb_priorityFT2, gbconstr);
        
        gbconstr.gridx = 1;
        gbconstr.gridy = 6;
        pInFlow2.add(sb_simSpeed2, gbconstr);

// 3th column: Actual values + units

        gbconstr.gridx = 2;
        gbconstr.gridy = 0;
        gbconstr.weightx = 0.;
        gbconstr.fill = GridBagConstraints.NONE;
        gbconstr.anchor = GridBagConstraints.WEST;

        //pClosedSystem.add(label_density = new Label(String.valueOf(dens_invkm) + str_vehperkm), gbconstr);
        //pRamp.add(label_qIn2 = new Label(String.valueOf(flow_invh) + str_vehperh), gbconstr);
        //pInFlow1.add(label_qIn3 = new Label(String.valueOf(flow_invh) + str_vehperh), gbconstr);
        //pLanechange.add(label_p_factor6 = new Label(String.valueOf(p_factor)), gbconstr);
        //////////////////////////////
        pInFlow1.add(label_qInFM = new Label(String.valueOf(inFlowFM) + str_vehperh), gbconstr);
        pInFlow2.add(label_qInFM1 = new Label(String.valueOf(inFlowFM1) + str_vehperh), gbconstr);
        ////////////////////////////////

        gbconstr.gridx = 2;
        gbconstr.gridy = 1;

        //pClosedSystem.add(label_perTr1 = new Label(String.valueOf((int) (100 * FRAC_TRUCK_INIT_CIRCLE)) + " %"), gbconstr);
        //pRamp.add(label_qRamp = new Label(String.valueOf(flowRamp_invh) + str_vehperh), gbconstr);
        //pInFlow1.add(label_perTr35 = new Label(String.valueOf(truckPerc) + " %"), gbconstr);
        //pLanechange.add(label_deltaB6 = new Label(String.valueOf(deltaB) + " m/s^2"), gbconstr);
        pInFlow1.add(label_qInFT1 = new Label(String.valueOf(inFlowFT) + str_vehperh), gbconstr);
        pInFlow2.add(label_qInFM2 = new Label(String.valueOf(inFlowFM2) + str_vehperh), gbconstr);

        gbconstr.gridx = 2;
        gbconstr.gridy = 2;
        //pClosedSystem.add(label_simSpeed1 = new Label(str_warpfactor + str_framerate), gbconstr);
        //pRamp.add(label_p_factor2 = new Label( //!! label_p_factorRamp String.valueOf(p_factor)), gbconstr); //!! p_factorRamp
        //pInFlow1.add(label_v0_limit = new Label(String.valueOf((int) (3.6 * v0_limit)) + " km/h"), gbconstr);
        //pLanechange.add(label_simSpeed6 = new Label(str_warpfactor + str_framerate), gbconstr);
        pInFlow1.add(label_qInFV1 = new Label(String.valueOf(inFlowFV) + str_vehperh), gbconstr);
        pInFlow2.add(label_qInFT2 = new Label(String.valueOf(inFlowFT) + str_vehperh), gbconstr);

        gbconstr.gridx = 2;
        gbconstr.gridy = 3;

        //pRamp.add(label_deltaB2 = new Label( //!! label_deltaBRampString.valueOf(deltaB) + " m/s^2"), gbconstr); //!! deltaBRamp
        //pInFlow1.add(label_p_factor35 = new Label(String.valueOf(p_factor)), gbconstr);
        pInFlow1.add(label_priorityFM1 = new Label(String.valueOf(priorFM) + str_veh), gbconstr);
        pInFlow2.add(label_qInFV2 = new Label(String.valueOf(inFlowFV) + str_vehperh), gbconstr);



        gbconstr.gridx = 2;
        gbconstr.gridy = 4;

        //pRamp.add(label_simSpeed2 = new Label(
        //      str_warpfactor + str_framerate), gbconstr);
        //pInFlow1.add(label_simSpeed3 = new Label(str_warpfactor + str_framerate), gbconstr);
        pInFlow1.add(label_priorityFT1 = new Label(String.valueOf(priorFT) + str_veh), gbconstr);
        pInFlow2.add(label_priorityFM2 = new Label(String.valueOf(priorFM) + str_veh), gbconstr);
        //pInFlow2.add(label_simSpeed2 = new Label(String.valueOf(str_warpfactor) + str_framerate), gbconstr);

        gbconstr.gridx = 2;
        gbconstr.gridy = 5;
        pInFlow1.add(label_simSpeed3 = new Label(str_warpfactor + str_framerate), gbconstr);
        pInFlow2.add(label_priorityFT2 = new Label(String.valueOf(priorFT) + str_veh), gbconstr);
        
        gbconstr.gridx = 2;
        gbconstr.gridy = 6;
        pInFlow2.add(label_simSpeed2 = new Label(String.valueOf(str_warpfactor) + str_framerate), gbconstr);
//###########################################
// martin jan05

        /*AdjustmentMethods.addScrollbar(pIDMparams, 1,
        sb_IDMv0, str_IDMv0, label_IDMv0);
        AdjustmentMethods.addScrollbar(pIDMparams, 2,
        sb_IDMa, str_IDMa, label_IDMa);
        AdjustmentMethods.addScrollbar(pIDMparams, 3,
        sb_IDMb, str_IDMb, label_IDMb);
        AdjustmentMethods.addScrollbar(pIDMparams, 4,
        sb_IDMT, str_IDMT, label_IDMT);
        AdjustmentMethods.addScrollbar(pIDMparams, 5,
        sb_IDMs1, str_IDMs1, label_IDMs1);*/


//###########################################


        pScrollbars.add("Panel1", pInFlow1);
        //pScrollbars.add("onRamp", pRamp);
        //pScrollbars.add("closedSystem", pClosedSystem);
        //pScrollbars.add("Intersection", pInFlow1);
        pScrollbars.add("Panel2", pInFlow2);
        //panel2.add("Intersectionp2", pInFlow2);
        //pResultados.add("colas", pColas);
        //pResultados.add("tiempos", pTiempos);
        //pScrollbars.add("Lanechange", pLanechange);*/

        addComponentListener(new ComponentAdapter() {

            public void componentResized(ComponentEvent e) {
                MicroSim.this.appletResized();
            }
        });
    } // end init

    public void appletResized() {
        System.out.println("appletResized:" + getSize().width + "," + getSize().height);
        makeGlobalLayout();
        simCanvas.setScales();
        setFonts();
    }

    private void setFonts() {
        int textHeight = (int) (1.0 * REL_TEXTHEIGHT * getSize().height);
        if (textHeight > MAX_TEXTHEIGHT) {
            textHeight = MAX_TEXTHEIGHT;
        }
        if (textHeight < MIN_TEXTHEIGHT) {
            textHeight = MIN_TEXTHEIGHT;
        }
        int sbFontHeight = (int) (SBTEXT_MAINTEXT_HEIGHTRATIO * textHeight);
        Font textFont = new Font("SansSerif", Font.PLAIN, textHeight);
        Font sbFont = new Font("SansSerif", Font.PLAIN, sbFontHeight);
        pButtons.setFont(textFont);
        pIDMparams.setFont(sbFont);
        pScrollbars.setFont(sbFont);
        //panel2.setFont(sbFont);
        System.out.println("textFont=" + textFont
                + ", sbFont=" + sbFont);
    }

    private void resetIDMlabels() {
        label_IDMv0.setText(String.valueOf(V0_INIT_KMH) + " km/h");
        label_IDMT.setText(String.valueOf(T_INIT_S) + " s");
        label_IDMa.setText(String.valueOf(A_INIT_CAR_MSII) + " m/s^2");
        label_IDMb.setText(String.valueOf(B_INIT_MSII) + " m/s^2");
        label_IDMs0.setText(String.valueOf(S0_INIT_M) + " m");
        label_IDMs1.setText(String.valueOf(S1_INIT_M) + " m");
    }

    private void makeGlobalLayout() {
        int usedWidth = getSize().width - 2 * MARGIN;
        int usedHeight = getSize().height - 4 * MARGIN;
        textHeight = (int) (REL_TEXTHEIGHT * getSize().width);

        System.out.println("******************************");
        System.out.println("MicroSim.makeGlobalLayout: "
                + "Applet-size=" + (getSize().width) + " X "
                + (getSize().height)
                + ", Client-size=" + usedWidth + " X "
                + usedHeight);


        int buttWinHeight = (int) (2.2 * textHeight + 3 * MARGIN);
        int buttWinTop = getInsets().top;

        int textWinHeight = (SHOW_TEXT) ? TEXTWINDOW_HEIGHT : 0;
        int textWinTop = (usedHeight + getInsets().top) - textWinHeight;

// not clear why I must subtract addtl. getInsets().top below
        int simWinHeight = usedHeight - buttWinHeight
                - textWinHeight - getInsets().top;
        int simWinTop = buttWinTop + buttWinHeight;
        int simSize = (simWinHeight < usedWidth) ? simWinHeight : usedWidth;

        int sbWidth = (int) (0.80 * simSize); // weiter unten auch veraendern!
        int sbWidthRes = (int) (0.40 * simSize);

        int sbHeight = (int) (0.3 * simWinHeight);

        int sb_IDMwidth = sbWidth; // as circular; no 2 cases!
        int sb_IDMheight = sbHeight;
        int sbLeft = (int) (0.5 * (simSize - sbWidth)); // 0.5 = center
        int sbLeftRes = (int) (1 * (simSize - sbWidth));

        int sbTop = simWinTop + (int) (1.5 * (simSize - 1.2 * sbHeight - sb_IDMheight));
        //int sbTopRes = simWinTop + (int) (1.2 * (simSize - 1.2 * sbHeight - sb_IDMheight));

        int sb_IDMtop = sbTop + (int) (1.2 * sbHeight);
        int sb_IDMleft = sbLeft + (int) (0.5 * (sbWidth - sb_IDMwidth));

        setLayout(null);

        pButtons.setBounds(getInsets().left, buttWinTop,
                usedWidth, buttWinHeight);
        this.add(pButtons);


        pScrollbars.setBounds(sbLeft, sbTop, sbWidth, sbHeight);
        //panel2.setBounds(sbLeftRes, sbTop, sbWidthRes, sbHeight);
        //pIDMparams.setBounds(sb_IDMleft, sb_IDMtop, sb_IDMwidth, sb_IDMheight);
        //pIDMparams.setBounds(sbLeft, sbTop, sbWidth, sbHeight);

//###########################################
// Martin jan05
//!!! nicht mal mit voellig neuem Panel geht dyn. Skalierung!!
//=> muesste die alten erst loeschen/uns. machen => Leck
// => vergiss es

// optional (un)sichtbarmachen;
//pIDMparams.setVisible(false);
//pIDMparams.setVisible(true);
//###########################################


        // <martin nov0> scrollbars flowcontrol und IDM hier deaktivieren!!
        // z.B. sinnvoll, IDM bars zu deaktivieren,
        // wenn Constants.SHOW_INSET_DIAG=true

        // paneles para los scrollbars y los parametros en el centro del lienzo
        this.add(pScrollbars);
        //this.add(panel2);
        //this.add(pIDMparams); // <martin nov0> scrollbar IDM hier

        if (choice_Scen == 7 || choice_Scen == 8) {
            //cardLayout.show(pScrollbars, "Intersection");
            
            cardLayout.show(pScrollbars, "Panel1");
        } else if (choice_Scen == 9 || choice_Scen == 10) {            
            //panel2.setVisible(true);
            //cardLayout.show(panel2, "Intersection2");
            cardLayout.show(pScrollbars, "Panel2");
        } else if (choice_Scen == 2) {
            cardLayout.show(pScrollbars, "onRamp");
        } else if (choice_Scen == 6) {
            cardLayout.show(pScrollbars, "Lanechange");
        } else {
            cardLayout.show(pScrollbars, "Panel1");
        }

        if (SHOW_TEXT) {
            textCanvas.setBounds(getInsets().left, textWinTop, usedWidth, textWinHeight);
            this.add(textCanvas);
        }
        simCanvas.setBounds(getInsets().left, simWinTop, usedWidth, simWinHeight);
        this.add(simCanvas);
    }
    // end makeGlobalLayout
    private final static boolean sbBroken =
            (new Scrollbar(Scrollbar.HORIZONTAL, 20, 10, 0, 20).getValue() != 20);

    private Scrollbar getSB(int min, int max, int init) {
        final int inc = 1;
        return new Scrollbar(Scrollbar.HORIZONTAL, init, inc, min,
                max + (sbBroken ? inc : 0));
    }

    public boolean handleEvent(Event evt) {
        //System.out.println("MicroSim.handleEvent(evt): evt.target ="+evt.target);
        switch (evt.id) {
            case Event.SCROLL_LINE_UP:
            case Event.SCROLL_LINE_DOWN:
            case Event.SCROLL_PAGE_UP:
            case Event.SCROLL_PAGE_DOWN:
            case Event.SCROLL_ABSOLUTE:
                int i_warpfactor = (int) (1000 * TIMESTEP_S / tsleep_ms); //!!Veraend Sb
                String str_warpfactor = String.valueOf(i_warpfactor)
                        + "."
                        + String.valueOf((int) (10000 * TIMESTEP_S / tsleep_ms - 10 * i_warpfactor));

                if (evt.target == sb_simSpeed1) {
                    tsleep_ms = (int) (Math.exp(-0.01 * sb_simSpeed1.getValue()));
                    label_simSpeed1.setText(
                            str_warpfactor + str_framerate);
                } else if (evt.target == sb_simSpeed2) {
                    tsleep_ms = (int) (Math.exp(-0.01 * sb_simSpeed2.getValue()));
                    label_simSpeed2.setText(
                            str_warpfactor + str_framerate);
                } else if (evt.target == sb_simSpeed3) {
                    tsleep_ms = (int) (Math.exp(-0.01 * sb_simSpeed3.getValue()));
                    label_simSpeed3.setText(
                            str_warpfactor + str_framerate);
                } else if (evt.target == sb_simSpeed6) {
                    tsleep_ms = (int) (Math.exp(-0.01 * sb_simSpeed6.getValue()));
                    label_simSpeed6.setText(
                            str_warpfactor + str_framerate);
                } else if (evt.target == sb_qIn2) {
                    int newval = sb_qIn2.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qIn) {
                        qIn = q;
                        label_qIn2.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } ///////////////////////////////////////////
                else if (evt.target == sb_qInFM) {
                    int newval = sb_qInFM.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qInFM) {
                        qInFM = q;
                        label_qInFM.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } else if (evt.target == sb_qInFM1) {
                    int newval = sb_qInFM1.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qInFM1) {
                        qInFM1 = q;
                        label_qInFM1.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } else if (evt.target == sb_qInFM2) {
                    int newval = sb_qInFM2.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qInFM2) {
                        qInFM2 = q;
                        label_qInFM2.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } else if (evt.target == sb_qInFT1) {
                    int newval = sb_qInFT1.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qInFT) {
                        qInFT = q;
                        label_qInFT1.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } else if (evt.target == sb_qInFV1) {
                    int newval = sb_qInFV1.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qInFV) {
                        qInFV = q;
                        label_qInFV1.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } else if (evt.target == sb_qInFT2) {
                    int newval = sb_qInFT2.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qInFT) {
                        qInFT = q;
                        label_qInFT2.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } else if (evt.target == sb_qInFV2) {
                    int newval = sb_qInFV2.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qInFV) {
                        qInFV = q;
                        label_qInFV2.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } else if (evt.target == sb_priorityFM1) {
                    int newval = sb_priorityFM1.getValue();
                    int p = newval;
                    if (p != priorFM) {
                        priorFM = p;
                        label_priorityFM1.setText(String.valueOf(newval)+str_veh);
                    }
                }else if (evt.target == sb_priorityFT1) {
                    int newval = sb_priorityFT1.getValue();
                    int p = newval;
                    if (p != priorFT) {
                        priorFT = p;
                        label_priorityFT1.setText(String.valueOf(newval)+str_veh);
                    }
                }
                        else if (evt.target == sb_priorityFM2) {
                    int newval = sb_priorityFM2.getValue();
                    int p = newval;
                    if (p != priorFM) {
                        priorFM = p;
                        label_priorityFM2.setText(String.valueOf(newval)+str_veh);
                    }
                }else if (evt.target == sb_priorityFT2) {
                    int newval = sb_priorityFT2.getValue();
                    int p = newval;
                    if (p != priorFT) {
                        priorFT = p;
                        label_priorityFT2.setText(String.valueOf(newval)+str_veh);
                    }
                }
                        ///////////////////////////////////////////
                else if (evt.target == sb_qIn3) {
                    int newval = sb_qIn3.getValue();
                    double q = ((double) (newval)) / 3600.0;
                    if (q != qIn) {
                        qIn = q;
                        label_qIn3.setText(String.valueOf(newval)
                                + str_vehperh);
                    }
                } else if (evt.target == sb_p_factorRamp) {
                    System.out.println("target=sb_p_factorRamp!!");
                    int newval = sb_p_factorRamp.getValue();
                    double p = newval / 100.0;
                    if (p != p_factorRamp) {
                        p_factorRamp = p;
                        label_p_factorRamp.setText(String.valueOf(p)
                                + " ");
                    }
                } else if (evt.target == sb_deltaBRamp) {
                    int newval = sb_deltaBRamp.getValue();
                    double p = newval / 100.0;
                    if (p != deltaBRamp) {
                        deltaBRamp = p;
                        label_deltaBRamp.setText(String.valueOf(p)
                                + " m/s^2");
                    }
                } else if (evt.target == sb_perTr1) {
                    int newval = sb_perTr1.getValue();
                    double pT = newval / 100.0;
                    if (pT != perTr) {
                        perTr = pT;
                        label_perTr1.setText(String.valueOf(newval) + " %");
                    }
                } else if (evt.target == sb_perTr35) {
                    int newval = sb_perTr35.getValue();
                    double pT = newval / 100.0;
                    if (pT != perTr) {
                        perTr = pT;
                        label_perTr35.setText(String.valueOf(newval) + " %");
                    }
                } else if (evt.target == sb_v0_limit) {
                    int newval = (int) (0.1 * sb_v0_limit.getValue() + 0.5);
                    v0_limit = 10 * newval / 3.6;
                    label_v0_limit.setText(String.valueOf(10 * newval) + " km/h");
                } else if (evt.target == sb_p_factor2) {
                    int newval = sb_p_factor2.getValue();
                    double p = newval / 100.0;
                    if (p != p_factor) {
                        p_factor = p;
                        label_p_factor2.setText(String.valueOf(p));
                    }
                    System.out.println("target=sb_p_factor2: p_factor =" + p);

                } else if (evt.target == sb_p_factor35) {
                    int newval = sb_p_factor35.getValue();
                    double p = newval / 100.0;
                    if (p != p_factor) {
                        p_factor = p;
                        label_p_factor35.setText(String.valueOf(p));
                    }
                    System.out.println("target=sb_p_factor35: p_factor =" + p);

                } else if (evt.target == sb_p_factor6) {
                    int newval = sb_p_factor6.getValue();
                    double p = newval / 100.0;
                    if (p != p_factor) {
                        p_factor = p;
                        label_p_factor6.setText(String.valueOf(p));
                    }
                } else if (evt.target == sb_deltaB2) {
                    int newval = sb_deltaB2.getValue();
                    double p = newval / 100.0;
                    if (p != deltaB) {
                        deltaB = p;
                        label_deltaB2.setText(String.valueOf(p) + " m/s^2");
                    }
                } else if (evt.target == sb_deltaB35) {
                    int newval = sb_deltaB35.getValue();
                    double p = newval / 100.0;
                    if (p != deltaB) {
                        deltaB = p;
                        label_deltaB35.setText(String.valueOf(p) + " m/s^2");
                    }
                } else if (evt.target == sb_deltaB6) {
                    int newval = sb_deltaB6.getValue();
                    double p = newval / 100.0;
                    if (p != deltaB) {
                        deltaB = p;
                        label_deltaB6.setText(String.valueOf(p) + " m/s^2");
                    }
                } //#####################################################
                //martin jan05
                else if (evt.target == sb_IDMv0) {
                    double newval = AdjustmentMethods.getVariableFromSliderpos(
                            sb_IDMv0.getValue(), V0_MIN_KMH, V0_MAX_KMH, false);
                    idmCar.v0 = newval / 3.6;
                    label_IDMv0.setText(String.valueOf((int) newval) + " km/h");
                    simCanvas.changeIDMCarparameters(idmCar);
                } else if (evt.target == sb_IDMT) {
                    idmCar.T = AdjustmentMethods.getVariableFromSliderpos(
                            sb_IDMT.getValue(), T_MIN_S, T_MAX_S, false);
                    String str_T = String.valueOf((int) idmCar.T)
                            + "." + String.valueOf(((int) (10 * idmCar.T)) % 10);
                    label_IDMT.setText(str_T + " s");
                    simCanvas.changeIDMCarparameters(idmCar);
                } else if (evt.target == sb_IDMa) {
                    idmCar.a = AdjustmentMethods.getVariableFromSliderpos(
                            sb_IDMa.getValue(), A_MIN_MSII, A_MAX_MSII, false);
                    String str_a = String.valueOf((int) idmCar.a)
                            + "." + String.valueOf(((int) (10 * idmCar.a)) % 10);
                    label_IDMa.setText(str_a + " m/s^2");
                    simCanvas.changeIDMCarparameters(idmCar);
                }


//############################################!#


                
                    //simCanvas.valuesIntersection(choice_Scen, qInFM, qIjFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB tsleep_ms);
                    simCanvas.valuesIntersection(choice_Scen, priorFM, priorFT, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);
                    //simCanvas.newValues2(qIn, perTr,
                    //      p_factor, deltaB, qRamp, p_factorRamp, deltaBRamp, tsleep_ms);
               

        }
        return super.handleEvent(evt);
    }

    public boolean action(Event event, Object arg) {
        System.out.println("MicroSim.action(...)");

        int i_warpfactor = (int) (1000 * TIMESTEP_S / tsleep_ms); //1000 new scen
        String str_warpfactor = String.valueOf(i_warpfactor)
                + "."
                + String.valueOf((int) (10000 * TIMESTEP_S / tsleep_ms - 10 * i_warpfactor));

        if (event.target == start_button) {
            stop(); // stop necessary!
            start();
            return true;
        }
        if (event.target == stop_button) {
            stop();
            return true;
        }

        if (event.target == button1) {
            stop();
            choice_Scen = 7; // Closed system

            //density = 0.001 * DENS_INIT_INVKM; // = 0.04
// (tsleep_ms not changed)

// comment out following 2 lines if no new values for truck perc
            //int truckPerc1 = (int) (100 * FRAC_TRUCK_INIT_CIRCLE); // = 100*0.005 = 0.5
            //perTr = truckPerc1 / 100.; // = 0.005

            //sb_density.setValue(DENS_INIT_INVKM); // 40
            //label_density.setText(
            //      String.valueOf(DENS_INIT_INVKM) + str_vehperkm);

            //sb_perTr1.setValue((int) (100 * perTr));
            //label_perTr1.setText(String.valueOf((int) (100 * perTr)) + " %");

            sb_simSpeed2.setValue((int) (100 * Math.log(1. / tsleep_ms)));
            label_simSpeed2.setText(str_warpfactor + str_framerate);

            resetIDMlabels();
            makeGlobalLayout();
            //cardLayout.show(pScrollbars, "Intersection");
cardLayout.show(pScrollbars, "Panel1");
            if (SHOW_TEXT) {
                textCanvas.update(choice_Scen);
            }
            //simCanvas.newValues(choice_Scen, 0, qIn, perTr, v0_limit, p_factor, deltaB, tsleep_ms);

            simCanvas.valuesIntersection(choice_Scen, priorFM, priorFT, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);
            //simCanvas.valuesIntersection(choice_Scen, nvehFM, nvehFT, nvehFV, nvehTM, nvehTT,nvehTV, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);

            simCanvas.start(choice_Scen);
            return true;
        }

        if (event.target == button2) {
            stop();
            choice_Scen = 8;

            //qIn = Q_INIT7 / 3600.;
            //qRamp = QRMP_INIT2 / 3600.;
            //int truckPerc2 = (int) (100 * FRAC_TRUCK_INIT);
            //perTr = truckPerc2 / 100.; // no slider bar in this scenario//!!!

            //System.out.println("truckPerc2=" + truckPerc2 + " perTr=" + perTr);

            //int flow2_invh = (int) (3600 * qIn);
            //int flowRamp2_invh = (int) (3600 * qRamp);
            //p_factorRamp = 0.;
            //deltaBRamp = DELTABRAMP_INIT; // negative shift threshold for onramp!
            //System.out.println("p_factorRamp=" + p_factorRamp);

            //sb_qIn2.setValue(flow2_invh);
            //label_qIn2.setText(String.valueOf(flow2_invh) + str_vehperh);

            //sb_qRamp.setValue(flowRamp2_invh);
            //label_qRamp.setText(String.valueOf(flowRamp2_invh) + str_vehperh);

            // !! sb_p_factorRamp, sb_deltaBRamp, label_*
            //sb_p_factor2.setValue((int) (100 * p_factor));
            //label_p_factor2.setText(String.valueOf(p_factor));

            //sb_deltaB2.setValue((int) (100 * deltaB));
            //label_deltaB2.setText(String.valueOf(deltaB));

            sb_simSpeed2.setValue((int) (100 * Math.log(1. / tsleep_ms)));
            label_simSpeed2.setText(str_warpfactor + str_framerate);

            resetIDMlabels();
            makeGlobalLayout();
            //cardLayout.show(pScrollbars, "Intersection");
cardLayout.show(pScrollbars, "Panel1");
            if (SHOW_TEXT) {
                textCanvas.update(choice_Scen);
            }

            //simCanvas.newValues(choice_Scen, 0, qIn, perTr, v0_limit, p_factor, deldaB, tsleep_ms);

            simCanvas.valuesIntersection(choice_Scen, priorFM, priorFT, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);
            //simCanvas.valuesIntersection(choice_Scen, nvehFM, nvexFT, nvehFV, nvehTM, nvehTT,nvehTV, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms)

            simCanvas.start(choice_Scen);
            return true;
        }

        if (event.target == button3) {


            stop();
            choice_Scen = 9; // Klosing of(one lane

            //p_factor = 0.25;
            //deltaB = 0.1;
            //v0^limit = V0_LIMIT_INIT_KMH / 3.6;
            //int flow3_invh = Q_INIT3;
            //aIn = flow3_invh / 3600.;
            //int truckPerc3 = (inv) (100 * FRAC_TRUCK_INIT);
            //perTr ="trugcPer�3 / 100.;*
            //sb_qIn3.setValue(flow3_invh);

            //label_qIn3.setText(String.valueOF(flow3Winvh) + str_vehpErh);

            //sb_perTr35.setValue(truckPerc3);
            //label_perTr35.setText(String.valueOf(truckPerc3) + " %");

            //sb_simSpeed3.setValue((int) (100 * Math.log(1. / tsleep_ms)));
            //label_simSpeed3.setText(str_warpfactor + str_framerate);

            //sb_v0_limit.setValue((int) (v0_limit * 3.6));
            //label_v0_limit.setText(String.valueOf(
            //      (int) (v0_limit * 3.6)) + " km/h");

            resetIDMlabels();
            makeGlobalLayout();

            if (SHOW_TEXT) {
                textCanvas.update(choice_Scen);
            }
            cardLayout.show(pScrollbars, "Panel2");

            //simCanvas.newValues(choice_Scen, 0, qIn, perTr, v0_limit, p_factor, deltaB, tsleep_ms);

            simCanvas.valuesIntersection(choice_Scen, priorFM, priorFT, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);
            //simCanvas.valuesIntersection(choice_Scen, nvehFM, nvehFT, nvehFV, nvehTM, nvehTT,nvehTV, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);

            simCanvas.start(choice_Scen);
            return true;
        }

        if (event.target == button4) {
            stop();
            choice_Scen = 10; // Ampliacion de ambos canales

            /*double p_factor = 0.25;
            double deltaB = 0.1;
            int flow4_invh = Q_INIT4;
            qIn = flow4_invh / 3600.;
            v0_limit = V0_INIT_KMH / 3.6;
            int truckPerc4 = (int) (100 * FRAC_TRUCK_INIT);
            perTr = truckPerc4 / 100.;
            
            sb_qIn3.setValue(flow4_invh);
            label_qIn3.setText(String.valueOf(flow4_invh) + str_vehperh);
            
            sb_perTr35.setValue(truckPerc4);
            label_perTr35.setText(String.valueOf(truckPerc4) + " %");
            
            sb_simSpeed3.setValue((int) (100 * Math.log(1. / tsleep_ms)));
            label_simSpeed3.setText(str_warpfactor + str_framerate);
            
            sb_v0_limit.setValue((int) (v0_limit * 3.6));
            label_v0_limit.setText(String.valueOf(
            (int) (v0_limit * 3.6)) + " km/h");*/



            resetIDMlabels();
            makeGlobalLayout();

            if (SHOW_TEXT) {
                textCanvas.update(choice_Scen);
            }

            cardLayout.show(pScrollbars, "Panel2");

            //simCanvas.newValues(choice_Scen, 0, qIn, perTr, v0_limit, p_factor, deltaB, tsleep_ms);

            simCanvas.valuesIntersection(choice_Scen, priorFM, priorFT, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);
            //simCanvas.valuesIntersection(choice_Scen, nvehFM, nvehFT, nvehFV, nvehTM, nvehTT,nvehTV, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);

            simCanvas.start(choice_Scen);
            return true;
        }

        if (event.target == button5) {
            stop();
            choice_Scen = 5; // Traffic lights

            int flow5_invh = Q_INIT5;
            qIn = flow5_invh / 3600.;
            int truckPerc5 = (int) (100 * FRAC_TRUCK_INIT);
            perTr = truckPerc5 / 100.;
            v0_limit = V0_INIT_KMH / 3.6;

            sb_qIn3.setValue(flow5_invh);
            label_qIn3.setText(String.valueOf(flow5_invh) + str_vehperh);

            sb_perTr35.setValue(truckPerc5);
            label_perTr35.setText(String.valueOf(truckPerc5) + " %");

            sb_simSpeed3.setValue((int) (100 * Math.log(1. / tsleep_ms)));
            label_simSpeed3.setText(str_warpfactor + str_framerate);

            sb_v0_limit.setValue((int) (v0_limit * 3.6));
            label_v0_limit.setText(String.valueOf(
                    (int) (v0_limit * 3.6)) + " km/h");



            resetIDMlabels();
            makeGlobalLayout();
            if (SHOW_TEXT) {
                textCanvas.update(choice_Scen);
            }

            cardLayout.show(pScrollbars, "Source");

            simCanvas.newValues(choice_Scen, 0,
                    qIn, perTr, v0_limit, p_factor, deltaB,
                    tsleep_ms);
            /*simCanvas.valoresDesdeRedoma(choice_Scen, density,
            qIn, perTr, v0_limit, p_factor, deltaB,
            tsleep_ms);*/
            simCanvas.start(choice_Scen);
            return true;
        }
        // <martin nov07>
        // Replace lane change slalom by application of perturbation!

        if (event.target == button6) {
            simCanvas.applyLocalPerturbation();
            return true;
        } /*
        if (event.target==button6){
        stop();
        choice_Szen=6; // Lanechange slalom
        
        p_factor=0.25;
        deltaB=0.1;
        
        sb_p_factor6.setValue( (int)(100*p_factor));
        label_p_factor6.setText (String.valueOf(p_factor));
        
        sb_deltaB6.setValue((int)(100*deltaB));
        label_deltaB6.setText (String.valueOf (deltaB)+" m/s^2");
        
        sb_simSpeed6.setValue((int)(100*Math.log(1./tsleep_ms)));
        label_simSpeed6.setText(str_warpfactor+str_framerate);
        
        resetIDMlabels();
        makeGlobalLayout();
        if(SHOW_TEXT){
        // textCanvas.setSZ(1);
        textCanvas.update(choice_Szen);
        }
        cardLayout.show(pScrollbars,"Lanechange");
        
        simCanvas.newValues(choice_Szen, density,
        qIn, perTr, v0_limit, p_factor, deltaB,
        tsleep_ms);
        simCanvas.start(choice_Szen,0.0);
        return true;
        }
         */ // </martin nov07>slalom disabled; replaced by perturbation
        else {
            return super.action(event, arg);
        }
    }

    public void start() {
        System.out.println("--------------------------------------------------");
        // TODO Spracheinstellungen und StartSzenario via parameter !! Arne
        System.out.println("MicroSim.start()");
        System.out.println(" check parameter \"language\" --> " + this.getParameter("language"));
        System.out.println(" check parameter \"scenario\" --> " + this.getParameter("scenario"));
        System.out.println("--------------------------------------------------");

        if (choice_Scen == 2) {
            simCanvas.newValues(choice_Scen, 0., qIn, perTr, v0_limit, 0., 0., tsleep_ms);
        } else {
            // se actualizan los valores de los parametros para la interseccion

            simCanvas.valuesIntersection(choice_Scen, priorFM, priorFT, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);
            //simCanvas.valuesIntersection(choice_Scen, nvehFM, nvehFT, nvehFV, nvehTM, nvehTT,nvehTV, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, v0, p_factor, deltaB, tsleep_ms);
        }


        makeGlobalLayout();
        if (true) {
            System.out.println("MicroSim.start()");
        }
        if (false) {
            System.out.print("start() => before simCanvas.start(choice_Szen=");
            System.out.println(choice_Scen + ", density=" + 0 + ")");
        }

        simCanvas.start(choice_Scen);
    }

    public void stop() {
        System.out.println("MicroSim.stop()");
        simCanvas.stop();
    }

    public void destroy() {
        stop();
    }

    private void setLanguage() {
        str_IDMv0 = lang.getDesiredVelIDM(); // velocidad inicial deseada
        str_IDMT = lang.getDesiredHeadwayIDM(); // 
        str_IDMa = lang.getDesiredAccelIDM();
        str_IDMb = lang.getDesiredDecelIDM();
        str_IDMs0 = lang.getMinGapIDM();
        str_IDMs1 = lang.getS1IDM();


        str_avgdens = lang.getAvgDensity();
        str_inflow = lang.getInflow();
        ////////////////////////////
        str_qInFM = lang.getInflowFM();
        str_qInFM1 = lang.getInflowFM1();
        str_qInFM2 = lang.getInflowFM2();

        str_qInFT = lang.getInflowFT();
        str_qInFV = lang.getInflowFV();
        ////////////////////////////
        str_priorityFM = lang.getPriorityFM();
        str_priorityFT = lang.getPriorityFT();
        ////////////////////////////
        str_rmpinflow = lang.getRampInflow();
        str_polite = lang.getMobilPoliteness();
        str_rmppolite = lang.getMobilPolitenessRamp();
        str_trucks = lang.getTruckPerc();
        str_db = lang.getMobilThreshold();
        str_rmpdb = lang.getMobilRampBias();
        str_speed = lang.getTimeWarp();
        str_framerate = lang.getFramerate();
        str_speedl = lang.getSpeedlimit();
        str_vehperh = lang.getVehPerHour();
        str_veh = lang.getVeh();
        str_cola = lang.getCola();
        str_vehperkm = lang.getVehPerKm();
        str_button1 = lang.getScenCurrentName();
        //str_button2 = lang.getScenOnrampName();
        str_button2 = lang.getScenToMeridaName();
        //str_button3 = lang.getScenToMeridaName();
        str_button3 = lang.getScenFromMeridaName();
        //str_button4 = lang.getScenUphillName();
        str_button4 = lang.getScenFromToMeridaName();
        str_button5 = lang.getScenTrafficlightsName();
        str_button6 = lang.getPerturbationButton();
    }
}
