package de.trafficsimulation;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Image;
import java.util.Vector;

// !!! Units of variables with names *_m in meter, ALL others in pixels!
// !!! pixel units: x: left->right; y: top->bottom!!
// !!! offset in x: xstart: pixel units = xstart + scale*m-units
public class SimCanvas extends Canvas implements Runnable, Constants {

    // Scaling and global size parameters
    // (determine with width() method AFTER cstr)
    /**
     *
     */
    private static final long serialVersionUID = 1L;
    private int xsize; // width, height of window in pixels
    private int ysize; // y pixel region of circle or U w/o on-ramp
    private double scale; // scale factor pixels/m
    private double scalex;
    private double scaley;
    // position and size of various road elements
    // (_m vars in m; others in pixels)
    private double rdlen_m;
    private double rdlen; // (not rounded) road length in pixel units
    private double rd_Merida_m;
    private double rd_Merida2_m;
    private double rd_FromTabay_m;
    private double rd_ToTabay_m;
    private double rd_Valle_m;
    public double rd_Merida;
    private double rd_Merida2;
    private double rd_FromTabay;
    private double rd_ToTabay;
    private double rd_Valle;
    private double straight1_m;// seccion desde la interseccion hasta la primera curva
    private double straight2_m;// desde la primera curva hasta la segunda curva
    private double straight3_m;// desde la segunda hasta el rio Mucujun
    private double interTT_m;
    private double straight1;
    private double straight2;
    private double straight3;
    private double interTT;
    private double l_straight_m; // length(m) of straight sections of Ushape / secciones rectas en forma de U
    private int l_straight;
    private double total_rmplen_m; // length of onramp + visible access road
    private double total_rmplen_m2;
    private int total_rmplen;
    private double ramppos_m; // center ramp position if straight road
    private double ramppos2_m; // posicion de la segunda rampa
    private int ramppos;
    private int ramppos2;
    private int lRamp;
    private int lw; // lane width
    private int r1, r2; // inner and outer radius  
    private int r11, r21; // inner and outer radius de la curva 1    
    private int r12, r22; // inner and outer radius de la curva 2
    private int xc1, yc1; // center de la curva 1    
    private int xc2, yc2; // center de la curva 2
    private int radius; // radius of road center
    private int roadMargin; // space road-margin
    //private int xc, yc; // center of SimCanvas (for all Szen)
    private int xstart, ystart; // if STRAIGHT_ROAD
    private int yrmpstart; // begin access road if STRAIGHT_ROAD
    private int vehWidth; // in pixels
    private int pkwLen; // in pixels
    private int lkwLen; // in pixels
    // control variables
    private int framesPerStep = FRAMES_PER_TIMESTEP; // tiempo del paso para la simulacion
    private static int fcnr = 0; // index of floating car; all
    // Moveable objects with
    // index=0 painted in same color (blue)
    // parameters influenced by the user; updated from the
    // corresponding variables of MicroSim* by method newValues(..)
    private int choice_Szen = 7; // {stopGo+free,obstacles,..,..,sync}
    private double density;
    private double qIn;
    //Flujo de entrada de cada canal
    private double qInFM; // from Merida
    private double qInFM1; // from Merida
    private double qInFM2; // from Merida
    private double qInFV; // from Valle, depende del flujo qInFM (from merida)
    private double qInFT; // from Tabay 
    private double perTr;
    private double perTrFM;
    private int priorFM = PRIORFM;
    private int priorFT;
    private double perTrFM1;
    private double perTrFM2;
    private double perTrFV;
    private double perTrFT;
    private double v0_limit;
    private double v0_limit_old = 0;
    private double p_factor;
    private double qRamp;
    private double qRamp2;
    private double p_factorRamp;
    private double p_factorRamp2;
    private double deltaB;
    private double deltaBRamp;
    private double deltaBRamp2;
    private int choice_Geom; // {Circle, U, Straight}
    private boolean circleCarsRemoved = false;
    // times
    private double time = 0.; // Simulation time (reset to 0 in each "run")
    private int itime = 0; // Accumulated simul. tinesteps
    private int itSinceLastChange = 0; // if displayed window changed
    // (itSinceLastChange==0) paint road grass etc anow, not only cars
    private int tsleep_ms;
    private int tsleepFrame_ms;
    private int simTime_ms;
    private int prepTime_ms;
    private int paintTime_ms;
    private int avgPaintTime_ms = 0;
    private int totalPaintTime_ms;
    private int totalTimeThisStep_ms;
    // Dynamical variables Array implementation
    //!!! rasius in m, dens_max in invkm; therefore 0.001!!!
    int imaxinit = (int) (0.001 * RADIUS_M * 4 * Math.PI * DENS_MAX_INVKM + 200);
    //imaxinit = (0.001*120*4*3,14*80+200) = 320,5
    // initial array size; increase if necessary (!!! yet to do!!!)
    double[] posTT = new double[imaxinit + 1];
    double[] posTV = new double[imaxinit + 1];
    double[] posFM = new double[imaxinit + 1];
    double[] posFM2 = new double[imaxinit + 1];
    double[] posFT = new double[imaxinit + 1];
    double[] posTM = new double[imaxinit + 1];
    double[] posFV = new double[imaxinit + 1];
    int[] laneTT = new int[imaxinit + 1];
    int[] laneTV = new int[imaxinit + 1];
    int[] laneFM = new int[imaxinit + 1];
    int[] laneFT = new int[imaxinit + 1];
    int[] laneTM = new int[imaxinit + 1];
    int[] laneFV = new int[imaxinit + 1];
    double[] lengthVehTT = new double[imaxinit + 1];
    double[] lengthVehTV = new double[imaxinit + 1];
    double[] lengthVehFM = new double[imaxinit + 1];
    double[] lengthVehFM2 = new double[imaxinit + 1];
    double[] lengthVehFT = new double[imaxinit + 1];
    double[] lengthVehTM = new double[imaxinit + 1];
    double[] lengthVehFV = new double[imaxinit + 1];
    int[] nrTT = new int[imaxinit + 1];
    int[] nrTV = new int[imaxinit + 1];
    int[] nrFM = new int[imaxinit + 1];
    int[] nrFT = new int[imaxinit + 1];
    int[] nrTM = new int[imaxinit + 1];
    int[] nrFV = new int[imaxinit + 1];
    int[] old_i = new int[imaxinit + 1]; // such that nr[i] = old_nr[old_i]
    double[] old_posTT = new double[imaxinit + 1];
    double[] old_posTV = new double[imaxinit + 1];
    double[] old_posFM = new double[imaxinit + 1];
    double[] old_posFM2 = new double[imaxinit + 1];
    double[] old_posFT = new double[imaxinit + 1];
    double[] old_posTM = new double[imaxinit + 1];
    double[] old_posFV = new double[imaxinit + 1];
    int[] old_laneTT = new int[imaxinit + 1];
    int[] old_laneTV = new int[imaxinit + 1];
    int[] old_laneFM = new int[imaxinit + 1];
    int[] old_laneFT = new int[imaxinit + 1];
    int[] old_laneTM = new int[imaxinit + 1];
    int[] old_laneFV = new int[imaxinit + 1];
    double[] posOn = new double[imaxinit + 1];
    double[] old_posOn = new double[imaxinit + 1];
    double[] lengthVehOn = new double[imaxinit + 1];
    // main objects
    private Object microstreetLock = new Object();
    private MicroStreet microstreet;
    public ToValle toValle;
    public ToTabay toTabay;
    public FromMerida fromMerida;
    public NewFromMerida fromMerida2;
    public FromTabay fromTabay;
    public ToMerida toMerida;
    public FromValle fromValle;
    private Image buffer;
    //private boolean painted=false;
    private Image diagram1, diagram2, diagram3, diagram4;
    private Color backgrColor = new Color(0, 150, 0); // green grass
    private Color roadColor = Color.lightGray;
    private Graphics bufferedGraphics = null;
    // private Graphics fd = null;
    private Thread runner = null;
    private int textHeight;
    private Font textFont;
    private FontMetrics metricsText;
    private Language lang;
    // Inset diagrams
    private int dia1x, dia2x, dia3x, dia4x, dia1y, dia2y, dia3y, dia4y;
    private int diaWidth, diaHeight;
    private TimeAvDet det1_0;
    private TimeAvDet det1_1;
    private SpaceAvDet det2_0;
    private SpaceAvDet det2_1;
    private TimeAvDet det3_0;
    private TimeAvDet det3_1;
    private SpaceAvDet det4_0;
    private SpaceAvDet det4_1;
    private TimeAvDet det5_0;
    private TimeAvDet det5_1;
    private SpaceAvDet det6_0;
    private SpaceAvDet det6_1;
    private TimeAvDet det7_0;
    private TimeAvDet det7_1;
    private SpaceAvDet det8_0;
    private SpaceAvDet det8_1;
    private StatBufDiag fcdxdv;
    private StatBufDiag Qrho;
    private StatBufDiag QrhoS;
    private StatBufDiag Qrho1;
    private StatBufDiag QrhoS1;
    private DynBufDiag fcvt;
    private DynBufDiag Qt;
    private int det2c = 0;
    private int det4c = 0;

    
    //simcanvas para la intersection
    public SimCanvas(int choice_Szen, double qInFM, double qInFM1, double qInFM2, double qInFT, double qInFV,
          double perTrFM, double perTrFT, double perTrFV, double p_factor, double deltaB, int tsleep_ms) {
    //public SimCanvas(int choice_Szen, int priorFM, int priorFT, double qInFM, double qInFM1, double qInFM2, double qInFT, double qInFV,
      //      double perTrFM, double perTrFT, double perTrFV, double p_factor, double deltaB, int tsleep_ms) {

        valuesIntersection(choice_Szen, priorFM, priorFT, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, 33, p_factor, deltaB, tsleep_ms);
        //valuesIntersection(choice_Szen, colaFM, colaFT, colaFV, colaTM, colaTT, colaTV, qInFM, qInFM1, qInFM2, qInFT, qInFV, perTrFM, perTrFT, perTrFV, 33, p_factor, deltaB, tsleep_ms);

        tsleepFrame_ms = 0;

        lang = Language.getInstance();

        fromMerida = new FromMerida(this, rd_Merida_m, 0., 0., choice_Szen, priorFM);
        if (choice_Szen == 9 || choice_Szen == 10) {
            fromMerida2 = new NewFromMerida(this, rd_Merida2_m, rd_Merida2_m, 0., 0., choice_Szen);
        }
        fromTabay = new FromTabay(this, rd_FromTabay, rd_FromTabay, p_factor, deltaB, choice_Szen);
        fromValle = new FromValle(this, rd_Valle_m, rd_Valle_m, p_factor, deltaB, choice_Szen);

        toTabay = new ToTabay(rd_ToTabay, 0., 0., choice_Szen);

        toMerida = new ToMerida(rd_Merida_m, p_factor, deltaB, choice_Szen);

        toValle = new ToValle(rd_Valle_m, 0., 0., choice_Szen);
    }

    public double getScale() {
        setScales();
        return (scale);
    }

    public void setScales() {

// global sizes: xsize,ysize=dimensions of the green background
        // from Dimension getSize() method;
// size() getSize()?
// does not work in constructor or directly in declaration

        int xsizeOld = xsize;
        int ysizeOld = ysize;
        xsize = getSize().width;
        ysize = getSize().height;

        if (xsize < 10) {
            xsize = 10;
        } // xsize=0 at beginning -> default!!
        if (ysize < 10) {
            ysize = 10;
        } // ysize=0 at beginning -> default!!

        if (buffer == null || xsizeOld != xsize || ysizeOld != ysize) {
            buffer = createImage(xsize, ysize);
        }

// determine physical road dimensions

        double xsize_m = 500;
        //double xsize2_m = 400;
        double ysize_m = 500;
        //double ysize2_m = 400;

        rd_Merida_m = STRAIGHT_FROMMERIDA_M;
        //rd_Merida2_m = STRAIGHT_FROMMERIDA_M + LARGO_CRUCE;
        rd_Valle_m = STRAIGHT_FROMTOVALLE_M;

        straight1_m = STRAIGHT1_M;
        straight2_m = STRAIGHT2_M;
        straight3_m = STRAIGHT3_M;
        interTT_m = INTERTT;
        rd_FromTabay_m = straight3_m + Math.PI * RADIUS_M + straight2_m + Math.PI * RADIUS_M + straight1_m;
        rd_ToTabay_m = interTT_m + straight3_m + Math.PI * RADIUS_M + straight2_m + Math.PI * RADIUS_M + straight1_m;

        if (choice_Szen == 8 || choice_Szen == 10) {
            rd_Valle_m = STRAIGHT_FROMTOVALLE_M + LARGO_CRUCE + rd_Merida_m;
        }
        if (choice_Szen == 9 || choice_Szen == 10) {
            rd_Merida2_m = STRAIGHT_FROMMERIDA_M + LARGO_CRUCE;
            rd_Merida2 = rd_Merida2_m * scale;
        }

        xsize_m = straight1_m;//-EXCESS_RDLEN_M;
        ysize_m = STRAIGHT_ASPECTRATIO * xsize_m;
        scalex = xsize / xsize_m;
        scaley = ysize / ysize_m;
        scale = scalex < scaley ? scalex : scaley;

// determine pixel road dimensions

        if (true) {
            straight1 = straight1_m * scale;
            straight2 = straight2_m * scale;
            straight3 = straight3_m * scale;
            interTT = interTT_m * scale;

            //rdlen = rdlen_m * scale;
            rd_Merida = rd_Merida_m * scale;
            rd_FromTabay = rd_FromTabay_m * scale;
            rd_ToTabay = rd_ToTabay_m * scale;
            rd_Valle = rd_Valle_m * scale;

            //total_rmplen = (int) (total_rmplen_m * scale);
            //lRamp = (int) (L_RAMP_M * scale);
            xstart = 0;//(int) (scale * EXCESS_RDLEN_M);
            ystart = (int) ((1. - STRAIGHT_ASPECTRATIO) * ysize);
            //ramppos_m = 100;// 600 m
            //ramppos = xstart + (int) (ramppos_m * scale);
            //yrmpstart = ystart
            //      + (int) (scale * ANGLE_ACCESS * (ramppos_m - 0.5 * L_RAMP_M));
            //l_straight = (int) (l_straight_m * scale);
            roadMargin = (int) (REL_ROAD_MARGIN * ysize);
            radius = (int) (RADIUS_M * scale);
            lw = (int) (LANEWIDTH_M * scale); // Lane width
            r1 = radius - lw; // inner radius
            r2 = radius + lw; // outer radius

            xc1 = (int) rd_Merida + 2 * lw + (int) straight1;// + roadMargin - (radius+lw); // center of semicircle 1
            yc1 = ystart + lw;//roadMargin + lw + radius; // (circle and U Scenarios)
            xc2 = xc1 - (int) straight2;
            yc2 = yc1 + r1 + 2 * lw + r1;

        } // end if true

        if (true) {
            System.out.println("SimCanvas.setScales:"
                    + " xsize_m= " + xsize_m
                    + " xsize= " + xsize
                    + " ysize= " + ysize
                    + " xstart= " + xstart
                    + " ystart= " + ystart
                    + " scale= " + scale
                    + " ystart/scale= " + ystart / scale);
        }

// vehicles (length varies from veh to vehicle)

        vehWidth = (int) (VEH_WIDTH_M * scale);
        pkwLen = (int) (PKW_LENGTH_M * scale);
        lkwLen = (int) (LKW_LENGTH_M * scale);

// text ("Car", "Truck", not that of scrollbars)

        textHeight = (int) (1.0 * REL_TEXTHEIGHT * getSize().height);
        if (textHeight > MAX_TEXTHEIGHT) {
            textHeight = MAX_TEXTHEIGHT;
        }
        if (textHeight < MIN_TEXTHEIGHT) {
            textHeight = MIN_TEXTHEIGHT;
        }

        textFont = new Font("SansSerif", Font.PLAIN, textHeight);
        metricsText = getFontMetrics(textFont);



        // layout of the insert diagrams

        double actsize = scale * 2 * RADIUS_M;

        diaHeight = (int) (0.275 * actsize);

        if (choice_Szen == 1) { // circle; stop&go + free
            diaWidth = (int) (0.275 * actsize);
            int xcenter = (int) (0.5 * actsize);
            dia1x = xcenter - diaWidth;
            dia1y = (int) (0.225 * actsize);
            dia2x = xcenter;
            dia2y = (int) (0.225 * actsize);
            dia3x = xcenter - diaWidth;
            dia3y = (int) (0.5 * actsize);
            dia4x = xcenter;
            dia4y = (int) (0.5 * actsize);
        }
        if (choice_Szen == 6) {
        }

        if (choice_Szen == 5) { // Ushape
            int xcenter = (int) (0.5 * ((0.9 * xsize) + (0.225 * actsize)));
            diaWidth = (int) (2 * (xcenter - (0.225 * actsize)));
            dia1x = xcenter - (int) (0.5 * diaWidth);
            dia1y = (int) (0.225 * actsize);
            dia2x = xcenter - (int) (0.5 * diaWidth);
            dia2y = (int) (0.5 * actsize);

        }
        if ((choice_Szen == 3) || (choice_Szen == 4)) { // Ushape
            int xcenter = (int) (0.5 * ((0.9 * xsize) + (0.225 * actsize)));
            diaWidth = (int) (xcenter - (0.225 * actsize));
            dia1x = xcenter - diaWidth;
            dia1y = (int) (0.225 * actsize);
            dia2x = xcenter;
            dia2y = (int) (0.225 * actsize);
            dia3x = xcenter - diaWidth;
            dia3y = (int) (0.5 * actsize);
            dia4x = xcenter;
            dia4y = (int) (0.5 * actsize);
        }

        // <martin aug06>

        boolean isRingroad = (choice_Szen == 1) || (choice_Szen == 6);

        diaWidth = (int) (0.40 * actsize);
        diaHeight = (int) (0.23 * actsize);

        dia1x = (isRingroad)
                ? (int) (0.99 * xsize) - 2 * diaWidth
                : (int) (0.80 * xsize) - 2 * diaWidth;
        dia2x = dia1x + diaWidth;
        dia3x = dia1x;
        dia4x = dia2x;

        dia1y = (isRingroad)
                ? (int) (0.80 * ysize) - 2 * diaHeight
                : (int) (0.85 * ysize) - 2 * diaHeight;
        dia2y = dia1y;
        dia3y = dia1y + diaHeight;
        dia4y = dia3y;
        // </Treiber>


    }
    // end setScales

    public void run() {

// long startTime = System.currentTimeMillis();

        System.out.println("SimCanvas.run()");

        //boolean traffLightTurnedGreen = false;
        time = 0.;

        setScales();
        if (SHOW_INSET_DIAG) {
            setInsetDiagrams();
        }

// The main loop: runs until current thread no longer runner;
// if stop or new scenario -> stop() method applied:
// sets runner=null -> run() method ends

        while (Thread.currentThread() == runner) {
//while ((Thread.currentThread()== runner) && (time<100)){
            try {
                Thread.sleep(tsleep_ms);
            } catch (InterruptedException e) {;
            }



// simulate new timestep

            long timeBeforeSim_ms = System.currentTimeMillis();
            synchronized (microstreetLock) {

                fromMerida.updateFromMerida(time, TIMESTEP_S, qInFM, qInFM1, perTrFM, priorFM);
                if (choice_Szen == 9 || choice_Szen == 10) {
                    fromMerida2.updateNewFromMerida(time, TIMESTEP_S, qInFM2, perTrFM, p_factor, deltaB);
                }
                fromTabay.updateFromTabay(time, TIMESTEP_S, qInFT, perTrFT, p_factor, deltaB, priorFT);
                fromValle.updateFromValle(time, TIMESTEP_S, qInFV, perTrFV, p_factor, deltaB, choice_Szen);
                toTabay.updateToTabay(time, TIMESTEP_S);
                toValle.updateToValle(time, TIMESTEP_S);
                toMerida.updateToMerida(time, TIMESTEP_S);


                if (SHOW_INSET_DIAG) {
                    updateInsetDiagrams();
                }
            }

// calculate some times

            time += TIMESTEP_S;
            itime++;
            itSinceLastChange += 1;

            simTime_ms = (int) (System.currentTimeMillis() - timeBeforeSim_ms);

            int dtf_ms = -1; //!!!time

            tsleepFrame_ms = (int) ((tsleep_ms - simTime_ms - prepTime_ms) / framesPerStep
                    - avgPaintTime_ms + dtf_ms);

            if (tsleepFrame_ms < 2) {
                tsleepFrame_ms = 2;
            }


// paint timestep
            // repaint() calls automatically this.update(g) -> paintSim(g);
// witg Graphics object g which need NOT be changed; do not
// change paint(), repaint method itself but only methods it
// calls automatically -> update

            repaint();

// totalTimeThisStep_ms should be the same or a little bit
// smaller as tsleep
// !! Cannot measure runtime of repaint -> 0 ms if
// times logged before and after repaint()

            totalTimeThisStep_ms = simTime_ms + prepTime_ms + totalPaintTime_ms;

            if (false) {
                System.out.println("tsleep=" + tsleep_ms
                        + ", tsleepFr=" + tsleepFrame_ms
                        + ", simT=" + simTime_ms
                        + ", prepT=" + prepTime_ms
                        + ", avgPaintT=" + avgPaintTime_ms
                        + ", totPaintT=" + totalPaintTime_ms
                        + ", totT=" + totalTimeThisStep_ms);
            }

        } // while (Thread.currentThread()== runner)

    }

    // called from start() method of main applet class MicroSim*
    public void start(int choice_Szen) {
        //public void start(int choice_Szen)
        System.out.println(
                "SimCanvas.start(choice_Szen=" + choice_Szen);

        fromMerida = new FromMerida(this, rd_Merida_m, 0., 0., choice_Szen, priorFM);
        if (choice_Szen == 9 || choice_Szen == 10) {
            fromMerida2 = new NewFromMerida(this, rd_Merida2_m, rd_Merida2_m, 0., 0., choice_Szen);

        }
        fromTabay = new FromTabay(this, rd_FromTabay_m, rd_FromTabay_m, p_factor, deltaB, choice_Szen);
        fromValle = new FromValle(this, rd_Valle_m, rd_Valle_m, 0, deltaB, choice_Szen);

        toValle = new ToValle(rd_Valle_m, 0., 0., choice_Szen);
        toTabay = new ToTabay(rd_ToTabay_m, 0., 0., choice_Szen);
        double rdToMerida = STRAIGHT_TOMERIDA_M;
        toMerida = new ToMerida(rdToMerida, 0., 0., choice_Szen);



        if (runner == null) {
            runner = new Thread(this);
            runner.start();

        }

    }

    // called from stop() method of main applet class MicroSim*
    public void applyLocalPerturbation() {
        microstreet.applyLocalPerturbation();
    }

    public void stop() {
        System.out.println("SimCanvas.stop()");
        if (runner != null) {
            Thread waitFor = runner;
            runner = null; // kill thread (seems necessary)
            // -> start=start from new
            try {
                waitFor.join(tsleep_ms);
            } catch (InterruptedException e) {
            }
        }
    }

    // update necessary!! repaint() [standard mechanism for animation]
    // calls per default update(g),
    // and the standard update(g) per default
    // clears everything before it calls
    // paint(); oversede with own update(g) which does not clear prior
    // to painting!
    // MicroApplet2_0
    public void update(Graphics g) {
        /*if (!DOUBLEBUF) {
         paintSim(g, framesPerStep);
         } else {*/
        paintBuffer(g);
        //}
    }

    // only called if display window changed or reactivated;
    // otherwise update() called for painting
    public void paint(Graphics g) {
        //System.out.println("SimCanvas.paint()");
        itSinceLastChange = 0;
        paintBuffer(g); // also if no double buffering!
    }

    private void paintBuffer(Graphics g) {

// make buffered graphics object bg "Doppelpufferung"

        if (buffer == null) {
            buffer = createImage(xsize, ysize);
        }
        bufferedGraphics = buffer.getGraphics();
        if (choice_Geom == 0) {
            paintCircle(bufferedGraphics);
        }
        if (choice_Geom == 1) {
            // aqui se llama al metodo paintInterseccion
            paintIntersection(bufferedGraphics);
            //paintU(bufferedGraphics);
        }
        if (choice_Geom == 2) {
            paintStraight(bufferedGraphics);
        }
        //if (choice_Geom==3){pintarInterseccion(bufferedGraphics);}
        paintSim(bufferedGraphics); // paint into buffer
        g.drawImage(buffer, 0, 0, this); // display buffer
    }

    // paint actual road simulation to Graphics object g
    // which may be screen or buffer
    // MicroApplet2_0
    private void paintSim(Graphics g) {
        paintSim(g, 1);
    }

    private void paintSim(Graphics g, int framesPerStep) {

        synchronized (microstreetLock) {

            if ((itSinceLastChange >= 1) && (time >= TIMESTEP_S)) {
                // en un principio itSinceLastChange=0, si la ventana cambia
                long timeBeforePrep_ms = System.currentTimeMillis();
                prepareVehData();
                prepTime_ms = (int) (System.currentTimeMillis() - timeBeforePrep_ms);

                long timeBeforePaint_ms = System.currentTimeMillis();
                if (choice_Geom == 0) {
                    updateCircle(g, framesPerStep);
                }
                if (choice_Geom == 1) {
                    // se llama al metodo updateIntersection(g,framesPerStep);
                    //updateU(g, framesPerStep);
                    updateIntersection(g, framesPerStep);
                }
                if (choice_Geom == 2) {
                    updateStraight(g, framesPerStep);
                }
                //if (choice_Geom==3){actualizarInterseccion(g,framesPerStep);}
                totalPaintTime_ms = (int) (System.currentTimeMillis() - timeBeforePaint_ms);
            } else {
                //!! repainting action after returning to applet;
// paint also the vehivles; therefore also the updte.. calls
                if (choice_Geom == 0) {
                    paintCircle(g);
                    updateCircle(g, 1);
                }
                if (choice_Geom == 1) {
                    paintIntersection(g);
                    updateIntersection(g, 1);
                    //paintU(g);
                    //updateU(g, 1);
                }
                if (choice_Geom == 2) {
                    paintStraight(g);
                    updateStraight(g, 1);
                }
                //if (choice_Geom==3){pintarInterseccion(g,1);}
            }

            paintLegend(g);
            paintTimes(g);
            paintSymbols(g);
            paintResultados(g);
            if (SHOW_INSET_DIAG) {
                paintInsetDiagrams(g);
            }
            if (((choice_Szen == 3) || (choice_Szen == 4) || (choice_Szen == 5))
                    && (v0_limit != v0_limit_old)) {
                imposeSpeedLimit(); // !!! Bring outside of graph; not confuse
// action with painting!!
                paintSpeedLimit(g);
            }
        }
    }

    // actualiza los parametros en cada paso time step
    private void updateIntersection(Graphics g, int framesPerStep) {
        System.out.println("in updateIntersection");


        Vector colorsFM = fromMerida.colors;
        Vector colorsTT = toTabay.colors;
        Vector colorsTV = toValle.colors;
        Vector colorsFT = fromTabay.colors;
        Vector colorsTM = toMerida.colors;
        Vector colorsFV = fromValle.colors;

        int imaxFM = colorsFM.size() - 1;
        System.out.println("in updateIntersection: FM= " + (imaxFM));
        int imaxFT = colorsFT.size() - 1;
        System.out.println("in updateIntersection: FT= " + (imaxFT));
        int imaxFV = colorsFV.size() - 1;
        System.out.println("in updateIntersection: FV= " + (imaxFV));
        int imaxTT = colorsTT.size() - 1;
        System.out.println("in updateIntersection: TT= " + (imaxTT));
        int imaxTV = colorsTV.size() - 1;
        System.out.println("in updateIntersection: TV= " + (imaxTV));
        int imaxTM = colorsTM.size() - 1;
        System.out.println("in updateIntersection: TM= " + (imaxTM));

        int sumPaintTime_ms = 0;

        for (int iframes = 1; iframes <= framesPerStep; iframes++) {

            long timeBeforePaint_ms = System.currentTimeMillis();

            double weightNew = (double) iframes / FRAMES_PER_TIMESTEP;
            double weightOld = (double) (iframes - 1) / FRAMES_PER_TIMESTEP;


// From Merida vehicles
            //if(choice_Szen == 7 || choice_Szen == 8 || choice_Szen == 9 || choice_Szen == 10)

            double yveh = ystart + 1.5 * lw;
            for (int i = imaxFM; i >= 0; i--) {
// old pos
                PolyVehFromMerida pvs = new PolyVehFromMerida(
                        weightNew * posFM[i] + (1. - weightNew) * old_posFM[i],
                        lengthVehFM[i], VEH_WIDTH_M, (int) (rd_Merida), yveh);

                PolyVehFromMerida pvs_old = new PolyVehFromMerida(
                        weightOld * posFM[i] + (1. - weightOld) * old_posFM[i] - 1,
                        lengthVehFM[i] + 2, VEH_WIDTH_M + 2, (int) (rd_Merida), yveh);

                int car_xPoints[] = pvs.xPoints;
                int car_yPoints[] = pvs.yPoints;
                int car_old_xPoints[] = pvs_old.xPoints;
                int car_old_yPoints[] = pvs_old.yPoints;


// paint old vehicle polygons with background color
                g.setColor(roadColor);
                g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);

// paint vehicle polygon at new position
                g.setColor((Color.red));
                g.fillPolygon(car_xPoints, car_yPoints, 4);

            }
            // Ampliacion From Merida
            if (choice_Szen == 9 || choice_Szen == 10) {

                Vector colorsFM2 = fromMerida2.colors;
                int imaxFM2 = colorsFM2.size() - 1;
                System.out.println("in updateIntersection: FM2= " + (imaxFM2));


                yveh = ystart + 1.5 * lw + lw;
                for (int i = imaxFM2; i >= 0; i--) {
// old pos

                    PolyVehFromMerida pvs = new PolyVehFromMerida(
                            weightNew * posFM2[i] + (1. - weightNew) * old_posFM2[i],
                            lengthVehFM2[i], VEH_WIDTH_M, (int) (rd_Merida2), yveh);

                    PolyVehFromMerida pvs_old = new PolyVehFromMerida(
                            weightOld * posFM2[i] + (1. - weightOld) * old_posFM2[i] - 1,
                            lengthVehFM2[i] + 2, VEH_WIDTH_M + 2, (int) (rd_Merida2), yveh);

                    int car_xPoints[] = pvs.xPoints;
                    int car_yPoints[] = pvs.yPoints;
                    int car_old_xPoints[] = pvs_old.xPoints;
                    int car_old_yPoints[] = pvs_old.yPoints;


// paint old vehicle polygons with background color
                    g.setColor(roadColor);
                    g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);

// paint vehicle polygon at new position
                    g.setColor((Color.blue));
                    g.fillPolygon(car_xPoints, car_yPoints, 4);

                }
            }

// From Valle vehicles
            for (int i = imaxFV; i >= 0; i--) {
// old pos

                int length = (int) (scale * lengthVehFV[i]);
                double pos_m = weightOld * posFV[i] + (1. - weightOld) * old_posFV[i];

                //if (choice_Szen == 7 || choice_Szen == 9) {
                PolyVehFromValle pvs = new PolyVehFromValle(
                        weightNew * posFV[i] + (1. - weightNew) * old_posFV[i],
                        lengthVehFV[i], VEH_WIDTH_M, (int) (rd_Valle));

                PolyVehFromValle pvs_old = new PolyVehFromValle(
                        weightOld * posFV[i] + (1. - weightOld) * old_posFV[i] - 1,
                        lengthVehFV[i] + 2, VEH_WIDTH_M + 2, (int) (rd_Valle));

                int car_xPoints[] = pvs.xPoints;
                int car_yPoints[] = pvs.yPoints;
                int car_old_xPoints[] = pvs_old.xPoints;
                int car_old_yPoints[] = pvs_old.yPoints;


// paint old vehicle polygons with background color
                g.setColor(roadColor);
                g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);

// paint vehicle polygon at new position
                g.setColor((Color.black));
                g.fillPolygon(car_xPoints, car_yPoints, 4);

                //}
                // pinta From Valle hasta la redoma, ampliacion  de To Merida
                /*if (choice_Szen == 8 || choice_Szen == 10) {
                 length = (int) (scale * lengthVehFV[i]);
                 pos_m = weightOld * posFV[i] + (1. - weightOld) * old_posFV[i];
                 int xstartFV = -(int) (scale * (pos_m - fromValle.roadLength));
                 //int xstartFV = -(int)((scale)*(pos_m+20));
                 g.setColor(roadColor);
                 g.fillRect(xstartFV, yTopFV, length, vehWidth);
                
                 // new pos
                
                 pos_m = weightNew * posFV[i] + (1. - weightNew) * old_posFV[i];
                 xstartFV = -(int) (scale * (pos_m - fromValle.roadLength));
                 //xstartFV = -(int)((scale)*(pos_m+20));
                 g.setColor((Color.white));
                 g.fillRect(xstartFV, yTopFV, length, vehWidth);
                 }*/
            }

// To Tabay vehicles

            for (int i = imaxTT; i >= 0; i--) {
                polyVehToTabay pvu = new polyVehToTabay(
                        weightNew * posTT[i] + (1. - weightNew) * old_posTT[i],
                        lengthVehTT[i], VEH_WIDTH_M, r1, r2, xc1, yc1, xc2, yc2,
                        (int) straight1, (int) straight2, (int) straight3, (int) rd_ToTabay);

                polyVehToTabay pvu_old = new polyVehToTabay(
                        weightOld * posTT[i] + (1. - weightOld) * old_posTT[i] - 1,
                        lengthVehTT[i] + 2, VEH_WIDTH_M + 2, r1, r2, xc1, yc1, xc2, yc2,
                        (int) straight1, (int) straight2, (int) straight3, (int) rd_ToTabay);

                int car_xPoints[] = pvu.xPoints;
                int car_yPoints[] = pvu.yPoints;
                int car_old_xPoints[] = pvu_old.xPoints;
                int car_old_yPoints[] = pvu_old.yPoints;

// paint old vehicle polygons with background color



                g.setColor(roadColor);
                g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);

// paint vehicle polygon at new position
                if (choice_Szen == 9 || choice_Szen == 10) {
                    g.setColor((Color.blue));
                } else {
                    g.setColor((Color.red));
                }
                g.fillPolygon(car_xPoints, car_yPoints, 4);
            }

// To Valle vehicles                
            for (int i = imaxTV; i >= 0; i--) {


                PolyVehToValle pvs = new PolyVehToValle(
                        weightNew * posTV[i] + (1. - weightNew) * old_posTV[i],
                        lengthVehTV[i], VEH_WIDTH_M, (int) (rd_Valle));

                PolyVehToValle pvs_old = new PolyVehToValle(
                        weightOld * posTV[i] + (1. - weightOld) * old_posTV[i] - 1,
                        lengthVehTV[i] + 2, VEH_WIDTH_M + 2, (int) (rd_Valle));

                int car_xPoints[] = pvs.xPoints;
                int car_yPoints[] = pvs.yPoints;
                int car_old_xPoints[] = pvs_old.xPoints;
                int car_old_yPoints[] = pvs_old.yPoints;


// paint old vehicle polygons with background color
                g.setColor(roadColor);
                g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);

// paint vehicle polygon at new position
                g.setColor((Color.red));
                g.fillPolygon(car_xPoints, car_yPoints, 4);
            }


// From Tabay vehicles

            for (int i = imaxFT; i >= 0; i--) {

                polyVehFromTabay pvu = new polyVehFromTabay(
                        weightNew * posFT[i] + (1. - weightNew) * old_posFT[i],
                        lengthVehFT[i], VEH_WIDTH_M,
                        r1, r2, xc1, yc1, xc2, yc2, (int) straight1, (int) straight2, (int) straight3, (int) (rd_FromTabay));

                polyVehFromTabay pvu_old = new polyVehFromTabay(
                        weightOld * posFT[i] + (1. - weightOld) * old_posFT[i] - 1,
                        lengthVehFT[i] + 2, VEH_WIDTH_M + 2,
                        r1, r2, xc1, yc1, xc2, yc2, (int) straight1, (int) straight2, (int) straight3, (int) (rd_FromTabay));

                int car_xPoints[] = pvu.xPoints;
                int car_yPoints[] = pvu.yPoints;
                int car_old_xPoints[] = pvu_old.xPoints;
                int car_old_yPoints[] = pvu_old.yPoints;

// paint old vehicle polygons with background color



                g.setColor(roadColor);
                g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);

// paint vehicle polygon at new position

                g.setColor((Color.white));
                g.fillPolygon(car_xPoints, car_yPoints, 4);

            }

// To Merida vehicles
            for (int i = imaxTM; i >= 0; i--) {
// old pos

                PolyVehToMerida pvs = new PolyVehToMerida(
                        weightNew * posTM[i] + (1. - weightNew) * old_posTM[i],
                        lengthVehTM[i], VEH_WIDTH_M, (int) (rd_Merida));

                PolyVehToMerida pvs_old = new PolyVehToMerida(
                        weightOld * posTM[i] + (1. - weightOld) * old_posTM[i] - 1,
                        lengthVehTM[i] + 2, VEH_WIDTH_M + 2, (int) (rd_Merida));

                int car_xPoints[] = pvs.xPoints;
                int car_yPoints[] = pvs.yPoints;
                int car_old_xPoints[] = pvs_old.xPoints;
                int car_old_yPoints[] = pvs_old.yPoints;


// paint old vehicle polygons with background color
                g.setColor(roadColor);
                g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);

// paint vehicle polygon at new position
                g.setColor((Color.white));
                g.fillPolygon(car_xPoints, car_yPoints, 4);
            }
            //}

            paintTime_ms = (int) (System.currentTimeMillis() - timeBeforePaint_ms);
            sumPaintTime_ms += paintTime_ms;
            try {
                Thread.sleep(tsleepFrame_ms);
            } catch (InterruptedException e) {
            };

        } // end frames loop


        avgPaintTime_ms = sumPaintTime_ms / framesPerStep;

        if (itime % NT_LINES == 0) {
            drawLines(g);
        }

    } // en updateIntersection

    // pinta la interseccion 
    private void paintIntersection(Graphics g) {

        g.setColor(backgrColor); //  VERDE
        g.fillRect(0, 0, xsize, ysize); // PINTA LA GRAMA EN VERDE

        int length = (int) rd_Merida;

        // pinta la ruta desde y hacia Merida
        g.setColor(roadColor);
        g.fillRect(xstart, ystart, length, lw + lw);

        // pinta la ruta desde y hacia El Valle
        int x1 = length + 2 * lw;// length es el tamaño del canal fromMerida
        int y1 = ystart;
        if (choice_Szen == 7 || choice_Szen == 8 || choice_Szen == 9 || choice_Szen == 10) {

            length = (int) (rd_Valle);
            if (choice_Szen == 8 || choice_Szen == 10) {
                length = (int) (rd_Valle - rd_Merida - LARGO_CRUCE);
            }

            g.setColor(roadColor);
            //g.fillRect(x1, y1 - lw, length, lw + lw);
            int x11, x12, x13, x14;

            // interseccion bajando de El Valle
            x11 = (int) rd_Merida;
            x13 = (int) (rd_Merida + LARGO_CRUCE);


            //Bajando de El Valle
            x11 = x11 + (int) LARGO_CRUCE;
            x13 = x13 + length;
            int[] poly2_x = {x11, x11, x13, x13};
            int[] poly2_y = {y1 - lw, y1 + lw, y1 - 0 * lw, y1 - 3 * lw};
            g.fillPolygon(poly2_x, poly2_y, poly2_x.length);

            // seccion de la interseccion bajando desde El Valle
            int[] poly3_x = {(int) (rd_Merida - CRUCE), (int) (rd_Merida - CRUCE), x11, x11};
            int[] poly3_y = {ystart + lw - lw, ystart + 2 * lw, ystart + lw, ystart - lw};
            g.fillPolygon(poly3_x, poly3_y, poly3_x.length);

            //Desde Merida a Tabay
            int[] poly4_x = {(int) rd_Merida, (int) rd_Merida, x1, x1};
            int[] poly4_y = {ystart + lw, ystart + lw + lw, ystart + 2 * lw + lw, ystart + lw + lw};
            g.fillPolygon(poly4_x, poly4_y, poly4_x.length);

            //pinta ampliacion del canal To Merida
            if (choice_Szen == 8 || choice_Szen == 10) {
                length = (int) (rd_Merida + 2 * lw);
                int x = 0;
                g.fillRect(x, y1 - lw, length, lw);

            }
            if (choice_Szen == 9 || choice_Szen == 10) {
                length = (int) (rd_Merida + 2 * lw);
                int x = 0;
                g.setColor(roadColor);
                g.fillRect(x, y1 + 2 * lw, length, lw);
            }

        }
        // seccion de la interseccion 
        //Desde Merida
        int[] poly2_x = {(int) rd_Merida, (int) rd_Merida, x1, x1};
        int[] poly2_y = {ystart + lw, ystart + 2 * lw, ystart + lw, ystart};
        g.fillPolygon(poly2_x, poly2_y, poly2_x.length);
        //Desde Tabay
        int[] poly3_x = {(int) rd_Merida, (int) rd_Merida, x1, x1};
        int[] poly3_y = {ystart, ystart + lw, ystart + 2 * lw, ystart + lw};
        g.fillPolygon(poly3_x, poly3_y, poly3_x.length);


        //Via hacia Tabay
        //Recta 1 desde la interseccion hasta la primera curva pronunciada 
        g.fillRect(x1, y1 + lw, (int) straight1, lw + lw);

        //Primera curva pronunciada

        // r1: es el radio interno
        // r2: es el radio externo
        g.fillOval(xc1 - r2, yc1, 2 * r2, 2 * r2);// g.fillOval(x,y,ancho,alto)
        g.setColor(backgrColor);
        g.fillOval(xc1 - r1, yc1 + 2 * lw, 2 * r1, 2 * r1);

        g.fillRect(x1 + (int) straight1 - (int) straight2, ystart + lw + 2 * r1, (int) straight2, 2 * r1); //verde

        //Recta 2 desde la primera curva pronunciada hasta la seginda curva
        g.setColor(roadColor);
        int x_straight2 = (int) rd_Merida + 2 * lw + ((int) straight1 - (int) straight2);
        int y_straight2 = ystart + r2 + 2 * r1;
        g.fillRect(x_straight2, y_straight2, (int) straight2, lw + lw);

        //Segunda curva
        g.fillOval(xc2 - r2, yc2, 2 * r2, 2 * r2);//g.fillOval(x,y,ancho,alto)

        g.setColor(backgrColor);
        g.fillOval(xc2 - r1, yc2 + 2 * lw, 2 * r1, 2 * r1);
        g.fillRect(x1 + (int) straight1 - (int) straight2, ystart + lw + 2 * r1 + 2 * lw + 2 * r1, (int) straight2, 2 * r1);//verde

        //Recta 3 desde la segunda curva pronunciada hasta antes del Rio Mucujun
        g.setColor(roadColor);
        int x_straight3 = (int) rd_Merida + 2 * lw + ((int) straight1 - (int) straight2);
        int y_straight3 = ystart + 3 * (r2);
        g.fillRect(x_straight3, y_straight3, (int) straight3, lw + lw);

        //drawLines(g);
    }

    // ################################################################
    // central simulation visulalization: paints street
    // and all vehicles for closed systems
    // First time: call paintCircle, paintU etc: Street, grass + vehicles
    // After this: call updateCircle etc: Only cars moved
    // ################################################################
    private void paintCircle(Graphics g) {
        /*
         g.setColor(backgrColor); // green grass
         g.fillRect(0, 0, xsize, ysize);
        
         //paintCircleRoad(g);
        
         g.setColor(roadColor); // grey circle with outer diameter
         g.fillOval(xc - r2, yc - r2, 2 * r2, 2 * r2);
         g.setColor(backgrColor); // green with inner diameter
         g.fillOval(xc - r1, yc - r1, 2 * r1, 2 * r1);
        
         g.setColor(roadColor); // pinta la ruta desde redoma
         g.fillRect(2 * xc, yc, 2 * lRamp, 2 * lw);
        
         g.setColor(roadColor); // pinta la ruta hacia redoma
         g.fillRect(2 * xc, yc - 3 * lw, 2 * lRamp, 2 * lw);
        
         g.setColor(roadColor); // pinta la ruta desde y hacia tabay
         g.fillRect(4 * xc, yc, lRamp, 2 * lw);
        
         g.setColor(roadColor); // pinta la ruta desde y hacia la culata
         g.fillRect(4 * xc, yc - 3 * lw, lRamp, 2 * lw);
        
         //actualizarVuelta(g, 1); // Actualiza el canal de la redoma hacia Tabay/La Culata
        
         drawLines(g);*/
    }

    private void paintCircleRoad(Graphics g) {
        System.out.println("Simcanvas.paintCircleRoad...");
        final int NPOINTS = 50;
        int[] xpoints = new int[2 * NPOINTS + 2];
        int[] ypoints = new int[2 * NPOINTS + 2];

        for (int i = 0; i < NPOINTS + 1; i++) {
            DetPosC dp = new DetPosC(rdlen_m * i / NPOINTS, LANEWIDTH_M);
            xpoints[i] = dp.x;
            ypoints[i] = dp.y;
        }
        for (int i = NPOINTS + 1; i < 2 * NPOINTS + 2; i++) {
            DetPosC dp = new DetPosC(rdlen_m * (i - 2 * NPOINTS - 1) / NPOINTS, -LANEWIDTH_M);
            xpoints[i] = dp.x;
            ypoints[i] = dp.y;
        }

        g.setColor(roadColor);
        //g.fillPolygon(xpoints, ypoints, 2*NPOINTS+2);
        drawLines(g);

    }

    private void updateCircle(Graphics g, int framesPerStep) {
        /*
         // calculate rectangles for new and old vehicle positions and draw.
         // Remove a little
         // bit more (make old rectangle larger); otherwise lost vehicle
         // pixels might appear; pos = back (!) edge of vehicle
         // same nr -> same length -> need no old length!
        
         Vector colors = microstreet.colors;
         int imax = colors.size() - 1;
         int sumPaintTime_ms = 0;
        
         for (int iframes = 1; iframes <= framesPerStep; iframes++) {
        
         long timeBeforePaint_ms = System.currentTimeMillis();
        
         double weightNew = (double) iframes / FRAMES_PER_TIMESTEP;
         double weightOld = (double) (iframes - 1) / FRAMES_PER_TIMESTEP;
        
         for (int i = imax; i >= 0; i--) {
         PolyVehCircle pvc = new PolyVehCircle(
         weightNew * pos[i] + (1. - weightNew) * old_pos[i],
         weightNew * lane[i] + (1. - weightNew) * old_lane[i],
         lengthVeh[i], VEH_WIDTH_M, r1, r2, xc, yc);
        
         // again, old veh a little big larger -> initpos-=1, length+=2
         PolyVehCircle pvc_old = new PolyVehCircle(
         weightOld * pos[i] + (1. - weightOld) * old_pos[i] - 1,
         weightOld * lane[i] + (1. - weightOld) * old_lane[i],
         lengthVeh[i] + 2, VEH_WIDTH_M + 2, r1, r2, xc, yc);
        
         int car_xPoints[] = pvc.xPoints;
         int car_yPoints[] = pvc.yPoints;
         int car_old_xPoints[] = pvc_old.xPoints;
         int car_old_yPoints[] = pvc_old.yPoints;
         //
         // paint old vehicle polygons with background color
        
         g.setColor(roadColor);
         g.fillPolygon(car_old_xPoints, car_old_yPoints, 4); // SE PINTAN LOS VEHICULOS EN LA VIEJA POSICION
         // CON EL COLOR DE LA RUTA: GRIS
        
         // paint vehicle polygon at new position
        
         if ((SHOW_INSET_DIAG) && (nr[i] == fcnr)) {
         g.setColor(Color.blue);
         } else {
         g.setColor((Color) (colors.elementAt(i)));
         }
         g.fillPolygon(car_xPoints, car_yPoints, 4); // PINTA LOS VEHICULOS EN LA NUEVA POSICION
         // EN COLOR ROJO
         }
        
         paintTime_ms = (int) (System.currentTimeMillis() - timeBeforePaint_ms);
         sumPaintTime_ms += paintTime_ms;
         try {
         Thread.sleep(tsleepFrame_ms);
         } catch (InterruptedException e) {
         };
        
         }
        
         avgPaintTime_ms = sumPaintTime_ms / framesPerStep;
        
         // remove old cars
        
         if ((itime % NT_ROAD == 0) && (circleCarsRemoved)) {
         //      paintCircleRoad(g);
         }
         if (itime % NT_LINES == 0) {
         drawLines(g);
         }
         */
    }

    private void paintStraight(Graphics g) {
        System.out.println("in paintStraight ...");
        g.setColor(backgrColor); // green grass everywhere
        g.fillRect(0, 0, xsize, ysize);
        g.setColor(roadColor); // straight road sections
        g.fillRect(xstart, ystart - 2 * lw, (int) rdlen, 2 * lw); // left-upper pt, obj size

        // draw on-ramp beginning at xc

        if (choice_Szen == 2) {
            g.setColor(roadColor);

// section parallel to main road
            int x_merge_end = (int) (ramppos + 0.5 * lRamp);
            System.out.println(" ramppos " + ramppos + " lRamp " + lRamp + "x_merge_end " + x_merge_end);
            g.fillRect(x_merge_end - lRamp, ystart, lRamp, lw);

// "merging taper" at end of on ramp
            int[] poly_x = {x_merge_end, x_merge_end, x_merge_end + 2 * lw};
            int[] poly_y = {ystart, ystart + lw, ystart};
            g.fillPolygon(poly_x, poly_y, poly_x.length);

// access road
            int[] poly2_x = {xstart, xstart, x_merge_end - lRamp, x_merge_end - lRamp};
            int[] poly2_y = {yrmpstart, yrmpstart + lw, ystart + lw, ystart};
            g.fillPolygon(poly2_x, poly2_y, poly2_x.length);

        }

        // lane-separating road lines

        drawLines(g);

        // speed-limit sign for Scen 3..5

        if ((choice_Szen == 3) || (choice_Szen == 4) || (choice_Szen == 5)) {
            paintSpeedLimit(g);
        }

    }

    private void paintU(Graphics g) {
        /*
         g.setColor(backgrColor); // green grass everywhere
         g.fillRect(0, 0, xsize, ysize);
        
         g.setColor(roadColor); // paint circle as in paintCircle
         g.fillOval(xc - r2, yc - r2, 2 * r2, 2 * r2);
         g.setColor(backgrColor);
         g.fillOval(xc - r1, yc - r1, 2 * r1, 2 * r1);
        
         g.fillRect(xc, 0, xsize, ysize); // let grass grow on right side
        
         g.setColor(roadColor); // two straight road sections
         g.fillRect(xc, yc - r2, l_straight, 2 * lw);
         g.fillRect(xc, yc + r1, l_straight, 2 * lw);
        
        
         // draw on-ramp for Szenario 2
        
         if (choice_Szen == 2) {
         g.setColor(roadColor);// para la rampa 1
         g.fillRect(0, yc + r2, xc + lRamp, lw);
        
         g.setColor(roadColor); // para la rampa 2
         g.fillRect(4 * xc, yc - r2 + 2 * lw, 2 * lRamp, lw);
        
         int[] poly_x = {xc + lRamp, xc + lRamp, xc + lRamp + 2 * lw};// para la rampa 1
         int[] poly_y = {yc + r2, yc + r2 + lw, yc + r2};
         g.fillPolygon(poly_x, poly_y, poly_x.length); // PINTA EL TRIANGULO AL FINAL DE LA RAMPA
        
         /*int[] poly_x2 = {xc+lRamp,xc+lRamp,xc+lRamp+2*lw};// para la rampa 2
         int[] poly_y2 = {10+5,10+5+lw,10+5};
         g.fillPolygon(poly_x2,poly_y2,poly_x2.length);*/
        //}

        // lane-separating road lines

        //drawLines(g);

        // speed-limit sign for Scen 3..5

        if ((choice_Szen == 3) || (choice_Szen == 4) || (choice_Szen == 5)) {
            paintSpeedLimit(g);
        }

    }

// se modifico para pintar los vehiculos de la rampa 2
    private void updateU(Graphics g, int framesPerStep) {

        /*
         Vector colors = microstreet.colors;
         int imax = colors.size() - 1; // imax = 100-1
         int sumPaintTime_ms = 0;
        
        
         // paint mainroad and possibly ramp vehicles in iframes steps
        
         for (int iframes = 1; iframes <= framesPerStep; iframes++) {
        
         long timeBeforePaint_ms = System.currentTimeMillis();
        
         double weightNew = (double) iframes / FRAMES_PER_TIMESTEP;
         double weightOld = (double) (iframes - 1) / FRAMES_PER_TIMESTEP;
        
         // main road vehicles
         //System.out.println("updateU: 3");
        
         for (int i = imax; i >= 0; i--) {
         PolyVehU pvu = new PolyVehU(
         weightNew * pos[i] + (1. - weightNew) * old_pos[i],
         weightNew * lane[i] + (1. - weightNew) * old_lane[i],
         lengthVeh[i], VEH_WIDTH_M, r1, r2, xc,
         l_straight, yc, (int) (rdlen));
        
         PolyVehU pvu_old = new PolyVehU(
         weightOld * pos[i] + (1. - weightOld) * old_pos[i] - 1,
         weightOld * lane[i] + (1. - weightOld) * old_lane[i],
         lengthVeh[i] + 2, VEH_WIDTH_M + 2,
         r1, r2, xc, l_straight, yc, (int) (rdlen));
        
         int car_xPoints[] = pvu.xPoints;
         int car_yPoints[] = pvu.yPoints;
         int car_old_xPoints[] = pvu_old.xPoints;
         int car_old_yPoints[] = pvu_old.yPoints;
        
         // paint old vehicle polygons with background color
        
        
        
         g.setColor(roadColor);
         g.fillPolygon(car_old_xPoints, car_old_yPoints, 5);// PINTA LOS VEHICULOS EN LA VIEJA POSICION EN GRIS
        
         // paint vehicle polygon at new position
        
         if ((SHOW_INSET_DIAG) && (nr[i] == fcnr)) {
         g.setColor(Color.blue);
         } else {
         g.setColor((Color) (colors.elementAt(i)));
         }
         g.fillPolygon(car_xPoints, car_yPoints, 5); // PINTA LOS VEHICULOS EN LA NUEVA POSICION EN ROJO
        
         }
        
        
         // if choice_Szen=2, paint ramp vehicles
        
         if (choice_Szen == 2) {
         int imaxOn = onramp.colors.size() - 1;// vector de tamaño 100-1
         // int imaxOn2 = onramp2.colors.size()-1;
         int yTopOn = yc + r2 + (int) (0.25 * lw);
         //int yTopOn2 = yc+r2-3*lw +(int)(0.25*lw);
        
         // paint new merging region
         // (otherwise paint of changed veh not removed)
        
         // if(choice_Szen==2){
         /* int xmerge = xc;
         int lmerge = lRamp;
         g.setColor(roadColor);
         g.fillRect(xmerge,yc+r2,lmerge,lw);*/

        /* int xmerge2 = 15;
         int lmerge2 = 40;
         g.setColor(roadColor);
         g.fillRect(xmerge,10+5,lmerge,lw);
         */
// }*/
/*
         for (int i = imaxOn; i >= 0; i--) {
        
         // old pos
        
         int length = (int) (scale * lengthVehOn[i]);
         double pos_m = weightOld * posOn[i] + (1. - weightOld) * old_posOn[i];
         int xstart = xc + lRamp
         + (int) (scale * (pos_m - onramp.roadLength));
         g.setColor(roadColor);
         g.fillRect(xstart, yTopOn, length, vehWidth);
        
         // new pos
        
         pos_m = weightNew * posOn[i] + (1. - weightNew) * old_posOn[i];
         xstart = xc + lRamp
         + (int) (scale * (pos_m - onramp.roadLength));
         g.setColor((Color) (onramp.colors.elementAt(i)));
         g.fillRect(xstart, yTopOn, length, vehWidth);
         }
         */
        /*//int imaxOn2 = onramp2.colors.size() - 1;
         //int yTopOn2 = yc + r2 - 3 * lw + (int) (0.25 * lw);
        
        
         for (int i = imaxOn2; i >= 0; i--) {
        
         // old pos
        
         int length2 = (int) (scale * lengthVehOn[i]);
         double pos_m2 = weightOld * posOn[i] + (1. - weightOld) * old_posOn[i];
         int xstart2 = 100 + (int) (scale * (pos_m2 - onramp2.roadLength));
         g.setColor(roadColor);
         g.fillRect(xstart2, yTopOn2, length2, vehWidth);
         // new pos
        
         pos_m2 = weightNew * posOn[i] + (1. - weightNew) * old_posOn[i];
         xstart2 = xc
         + (int) (scale * (pos_m2 - onramp2.roadLength));
         g.setColor((Color) (onramp2.colors.elementAt(i)));
         g.fillRect(xstart2, yTopOn2, length2, vehWidth);
         }*/
        /*            }
        
         paintTime_ms = (int) (System.currentTimeMillis() - timeBeforePaint_ms);
         sumPaintTime_ms += paintTime_ms;
         try {
         Thread.sleep(tsleepFrame_ms);
         } catch (InterruptedException e) {
         };
        
         }
         avgPaintTime_ms = sumPaintTime_ms / framesPerStep;
        
         if (itime % NT_LINES == 0) {
         drawLines(g);
         }
         */
    } // end updateU

    private void updateStraight(Graphics g, int framesPerStep) {
        /*    System.out.println("in updateStraight");
        
        
         Vector colors = microstreet.colors;
         int imax = colors.size() - 1;
         int sumPaintTime_ms = 0;
        
        
         // paint mainroad and possibly ramp vehicles in iframes steps
        
         for (int iframes = 1; iframes <= framesPerStep; iframes++) {
        
         long timeBeforePaint_ms = System.currentTimeMillis();
        
         double weightNew = (double) iframes / FRAMES_PER_TIMESTEP;
         double weightOld = (double) (iframes - 1) / FRAMES_PER_TIMESTEP;
        
        
         // main road vehicles
        
         for (int i = imax; i >= 0; i--) {
         PolyVehStraight pvs = new PolyVehStraight(
         weightNew * pos[i] + (1. - weightNew) * old_pos[i],
         weightNew * lane[i] + (1. - weightNew) * old_lane[i],
         lengthVeh[i], VEH_WIDTH_M, (int) (rdlen));
        
         PolyVehStraight pvs_old = new PolyVehStraight(
         weightOld * pos[i] + (1. - weightOld) * old_pos[i] - 1,
         weightOld * lane[i] + (1. - weightOld) * old_lane[i],
         lengthVeh[i] + 2, VEH_WIDTH_M + 2, (int) (rdlen));
        
         int car_xPoints[] = pvs.xPoints;
         int car_yPoints[] = pvs.yPoints;
         int car_old_xPoints[] = pvs_old.xPoints;
         int car_old_yPoints[] = pvs_old.yPoints;
        
         // paint old vehicle polygons with background color
        
        
        
         g.setColor(roadColor);
         g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);
        
         // paint vehicle polygon at new position
        
         if ((SHOW_INSET_DIAG) && (nr[i] == fcnr)) {
         g.setColor(Color.blue);
         } else {
         g.setColor((Color) (colors.elementAt(i)));
         }
         g.fillPolygon(car_xPoints, car_yPoints, 4);
         }
        
        
         // if choice_Szen=2, paint ramp vehicles
        
         if (choice_Szen == 2) {
         int imaxOn = onramp.colors.size() - 1; // vector de tamaño 100-1 heredado de MicroStreet
         g.setColor(roadColor);
         //int imaxOn2 = onramp2.colors.size() - 1;
         // paint new merging region
         // (otherwise paint of changed veh not removed)
        
        
         g.setColor(roadColor);
         int x_merge_end = (int) (ramppos + 0.5 * lRamp);
         g.fillRect(x_merge_end - lRamp - 10, ystart, lRamp + 10, lw);
        
         for (int i = imaxOn; i >= 0; i--) {
         PolyVehStraightRamp pvs = new PolyVehStraightRamp(
         weightNew * posOn[i] + (1. - weightNew) * old_posOn[i],
         lengthVehOn[i], VEH_WIDTH_M, (int) (total_rmplen));
        
         PolyVehStraightRamp pvs_old = new PolyVehStraightRamp(
         weightOld * posOn[i] + (1. - weightOld) * old_posOn[i] - 1,
         lengthVehOn[i] + 2, VEH_WIDTH_M + 2, (int) (total_rmplen));
        
         int car_xPoints[] = pvs.xPoints;
         int car_yPoints[] = pvs.yPoints;
         int car_old_xPoints[] = pvs_old.xPoints;
         int car_old_yPoints[] = pvs_old.yPoints;
        
        
         // paint old vehicle polygons with background color
         g.setColor(roadColor);
         g.fillPolygon(car_old_xPoints, car_old_yPoints, 4);
        
         // paint vehicle polygon at new position
        
         if ((SHOW_INSET_DIAG) && (nr[i] == fcnr)) {
         g.setColor(Color.blue);
         } else {
         g.setColor((Color) (onramp.colors.elementAt(i)));
         }
         g.fillPolygon(car_xPoints, car_yPoints, 4);
         }
        
         } // end choice_Szen=2, Straight road
        
         paintTime_ms = (int) (System.currentTimeMillis() - timeBeforePaint_ms);
         sumPaintTime_ms += paintTime_ms;
         try {
         Thread.sleep(tsleepFrame_ms);
         } catch (InterruptedException e) {
         };
        
         } // end frames loop
        
         avgPaintTime_ms = sumPaintTime_ms / framesPerStep;
        
         if (itime % NT_LINES == 0) {
         drawLines(g);
         }
        
         */ } // end updateStraight

    private void prepareVehData() {


        // vectores que almacenan los datos actuales (TT: toTabay)
        Vector positionsTT = toTabay.positions;
        Vector numbersTT = toTabay.numbers;
        Vector lanesTT = toTabay.lanes;
        Vector lengthsTT = toTabay.lengths;
        // vectores que almacenan los viejos datos (TT: toTabay)
        Vector old_positionsTT = toTabay.old_pos;
        Vector old_lanesTT = toTabay.old_lanes;
        Vector old_numbersTT = toTabay.old_numbers;

        // vectores que almacenan los datos actuales (TV: toValle)
        Vector positionsTV = toValle.positions;
        Vector numbersTV = toValle.numbers;
        Vector lanesTV = toValle.lanes;
        Vector lengthsTV = toValle.lengths;
        // vectores que almacenan los viejos datos (TV: toValle)        
        Vector old_positionsTV = toValle.old_pos;
        Vector old_lanesTV = toValle.old_lanes;
        Vector old_numbersTV = toValle.old_numbers;

        // vectores que almacenan los datos actuales (FM: fromMerida)
        Vector positionsFM = fromMerida.positions;
        Vector numbersFM = fromMerida.numbers;
        Vector lanesFM = fromMerida.lanes;
        Vector lengthsFM = fromMerida.lengths;
        // vectores que almacenan los viejos datos (FM: fromMerida)        
        Vector old_positionsFM = fromMerida.old_pos;
        Vector old_lanesFM = fromMerida.old_lanes;
        Vector old_numbersFM = fromMerida.old_numbers;


        // vectores que almacenan los datos actuales (FT: FromTabay)
        Vector positionsFT = fromTabay.positions;
        Vector numbersFT = fromTabay.numbers;
        Vector lanesFT = fromTabay.lanes;
        Vector lengthsFT = fromTabay.lengths;
        // vectores que almacenan los viejos datos (FT: FromTabay)
        Vector old_positionsFT = fromTabay.old_pos;
        Vector old_lanesFT = fromTabay.old_lanes;
        Vector old_numbersFT = fromTabay.old_numbers;

        // vectores que almacenan los datos actuales (FV: FromValle)
        Vector positionsFV = fromValle.positions;
        Vector numbersFV = fromValle.numbers;
        Vector lanesFV = fromValle.lanes;
        Vector lengthsFV = fromValle.lengths;
        // vectores que almacenan los viejos datos (FV: FromValle)
        Vector old_positionsFV = fromValle.old_pos;
        Vector old_lanesFV = fromValle.old_lanes;
        Vector old_numbersFV = fromValle.old_numbers;

        // vectores que almacenan los datos actuales (TM: ToMerida)
        Vector positionsTM = toMerida.positions;
        Vector numbersTM = toMerida.numbers;
        Vector lanesTM = toMerida.lanes;
        Vector lengthsTM = toMerida.lengths;
        // vectores que almacenan los viejos datos (TM: toMerida)
        Vector old_positionsTM = toMerida.old_pos;
        Vector old_lanesTM = toMerida.old_lanes;
        Vector old_numbersTM = toMerida.old_numbers;

        int imaxTT = positionsTT.size() - 1;
        int imaxTV = positionsTV.size() - 1;
        int imaxFM = positionsFM.size() - 1;
        int imaxFM2 = 0;
        if (choice_Szen == 9 || choice_Szen == 10) {
            Vector positionsFM2 = fromMerida2.positions;
            imaxFM2 = positionsFM2.size() - 1;
        }
        int imaxFT = positionsFT.size() - 1;
        int imaxTM = positionsTM.size() - 1;
        int imaxFV = positionsFV.size() - 1;


        if (imaxTT > imaxinit) {
            System.out.println("imax>imaxinit!! Doubling array size...");
        }
        if (imaxTV > imaxinit) {
            System.out.println("imax>imaxinit!! Doubling array size...");
        }
        if (imaxFM > imaxinit) {
            System.out.println("imax>imaxinit!! Doubling array size...");
        }
        if (choice_Szen == 9) {
            if (imaxFM2 > imaxinit) {
                System.out.println("imax>imaxinit!! Doubling array size...");
            }
        }
        if (imaxFT > imaxinit) {
            System.out.println("imax>imaxinit!! Doubling array size...");
        }
        if (imaxTM > imaxinit) {
            System.out.println("imax>imaxinit!! Doubling array size...");
        }
        if (imaxFV > imaxinit) {
            System.out.println("imax>imaxinit!! Doubling array size...");
        }
        // define the arrays

        for (int i = imaxTT; i >= 0; i--) {

            posTT[i] = ((Double) positionsTT.elementAt(i)).doubleValue();
            old_posTT[i] = ((Double) old_positionsTT.elementAt(i)).doubleValue();
            //laneTT[i] = ((Integer) lanesTT.elementAt(i)).intValue();
            lengthVehTT[i] = ((Double) lengthsTT.elementAt(i)).doubleValue();
            //nrTT[i] = ((Integer) numbersTT.elementAt(i)).intValue();
        }

        for (int i = imaxTV; i >= 0; i--) {

            posTV[i] = ((Double) positionsTV.elementAt(i)).doubleValue();
            old_posTV[i] = ((Double) old_positionsTV.elementAt(i)).doubleValue();
            //laneTV[i] = ((Integer) lanesTV.elementAt(i)).intValue();
            lengthVehTV[i] = ((Double) lengthsTV.elementAt(i)).doubleValue();
            //nrTV[i] = ((Integer) numbersTV.elementAt(i)).intValue();
        }

        for (int i = imaxFM; i >= 0; i--) {

            posFM[i] = ((Double) positionsFM.elementAt(i)).doubleValue();
            old_posFM[i] = ((Double) old_positionsFM.elementAt(i)).doubleValue();
            //laneFM[i] = ((Integer) lanesFM.elementAt(i)).intValue();
            lengthVehFM[i] = ((Double) lengthsFM.elementAt(i)).doubleValue();
            //nrFM[i] = ((Integer) numbersFM.elementAt(i)).intValue();
        }
        if (choice_Szen == 9 || choice_Szen == 10) {
            for (int i = imaxFM2; i >= 0; i--) {
                Vector positionsFM2 = fromMerida2.positions;
                Vector lengthsFM2 = fromMerida2.lengths;
                // vectores que almacenan los viejos datos (FM: fromMerida)        
                Vector old_positionsFM2 = fromMerida2.old_pos;
                posFM2[i] = ((Double) positionsFM2.elementAt(i)).doubleValue();
                old_posFM2[i] = ((Double) old_positionsFM2.elementAt(i)).doubleValue();
                //laneFM[i] = ((Integer) lanesFM.elementAt(i)).intValue();
                lengthVehFM2[i] = ((Double) lengthsFM2.elementAt(i)).doubleValue();
                //nrFM[i] = ((Integer) numbersFM.elementAt(i)).intValue();
            }
        }

        for (int i = imaxFT; i >= 0; i--) {

            posFT[i] = ((Double) positionsFT.elementAt(i)).doubleValue();
            old_posFT[i] = ((Double) old_positionsFT.elementAt(i)).doubleValue();
            //laneTT[i] = ((Integer) lanesTT.elementAt(i)).intValue();
            lengthVehFT[i] = ((Double) lengthsFT.elementAt(i)).doubleValue();
            //nrTT[i] = ((Integer) numbersTT.elementAt(i)).intValue();
        }

        for (int i = imaxTM; i >= 0; i--) {

            posTM[i] = ((Double) positionsTM.elementAt(i)).doubleValue();
            old_posTM[i] = ((Double) old_positionsTM.elementAt(i)).doubleValue();
            //laneTT[i] = ((Integer) lanesTT.elementAt(i)).intValue();
            lengthVehTM[i] = ((Double) lengthsTM.elementAt(i)).doubleValue();
            //nrTT[i] = ((Integer) numbersTT.elementAt(i)).intValue();
        }

        for (int i = imaxFV; i >= 0; i--) {

            posFV[i] = ((Double) positionsFV.elementAt(i)).doubleValue();
            old_posFV[i] = ((Double) old_positionsFV.elementAt(i)).doubleValue();
            //laneFV[i] = ((Integer) lanesFV.elementAt(i)).intValue();
            lengthVehFV[i] = ((Double) lengthsFV.elementAt(i)).doubleValue();
            //nrFV[i] = ((Integer) numbersFV.elementAt(i)).intValue();
        }

        // on-ramp data

        /*if (choice_Szen == 2) {
         for (int i = imaxOn; i >= 0; i--) {
         posOn[i] = ((Double) onramp.positions.elementAt(i)).doubleValue();
         old_posOn[i] = ((Double) onramp.old_pos.elementAt(i)).doubleValue();
         lengthVehOn[i] = ((Double) onramp.lengths.elementAt(i)).doubleValue();
         }
         }*/

    }

    // draw lane-separating road lines
    private void drawLines(Graphics g) {

        g.setColor(Color.white);
        int lineLength = (int) (scale * LINELENGTH_M);
        int gapLength = (int) (scale * GAPLENGTH_M);
        int lineWidth = 1;
        // nuber of lines: determ with rdlen/(lineLength+..)
        // would imply large discr. error!!
        int numberOfLines = (int) (rdlen / (scale * (LINELENGTH_M + GAPLENGTH_M)));
        //System.out.println("rdlen="+rdlen+" scale="+scale+
        // " numberOfLines="+numberOfLines);

        if (choice_Geom == 0) {
            for (int i = 0; i < numberOfLines; i++) {
                double pos_m = i * (LINELENGTH_M + GAPLENGTH_M);
                // PolyVehCircle roadLine = new PolyVehCircle(pos_m, -1, lineLength, lineWidth, r1, r2, xc, yc);
                // g.fillPolygon(roadLine.xPoints, roadLine.yPoints, 4);
            }
        }

        if (choice_Geom == 1) {
            /*for (int i = 0; i < numberOfLines; i++) {
             double pos_m = i * (LINELENGTH_M + GAPLENGTH_M);
             PolyVehU roadLine = new PolyVehU(pos_m, -1, lineLength, lineWidth, r1, r2,
             xc, l_straight, yc, (int) (rd_Tabay));
             g.fillPolygon(roadLine.xPoints, roadLine.yPoints, 4);
             }*/
            if (choice_Szen == 7) {
                numberOfLines = (int) (STRAIGHT_FROMTOVALLE_M / (LINELENGTH_M + GAPLENGTH_M) + 1);
                for (int i = 0; i < numberOfLines; i++) {
                    //int xstart = xc + (int) (i * scale * (LINELENGTH_M + GAPLENGTH_M));
// drawPolygon draws centered lines -> must add half linewidth
// to height,width parameters of drawRect
                    //int yTop = yc + r2 - (int) ((lineWidth + 1) / 2);
                    //int xstartFV = (int) (i*scale * (fromValle.roadLength-fromMerida.length()));
                    //int yTopFV = yc - (int)(lw) + (int) ((lineWidth + 1)/2);
                    //g.fillRect(xstart, yTopFV, lineLength + 1, lineWidth);
                }
            }
        }


        if (choice_Geom == 2) {
            for (int i = 0; i < numberOfLines; i++) {
                double pos_m = i * (LINELENGTH_M + GAPLENGTH_M);
                PolyVehStraight roadLine = new PolyVehStraight(pos_m, -1, lineLength, lineWidth, (int) (rdlen));
                g.fillPolygon(roadLine.xPoints, roadLine.yPoints, 4);
            }
            if (choice_Szen == 2) {
                numberOfLines = (int) (L_RAMP_M / (LINELENGTH_M + GAPLENGTH_M) + 1);
                for (int i = 0; i < numberOfLines; i++) {
                    int xLeft = ramppos
                            + (int) (scale * (-0.5 * L_RAMP_M + i * (LINELENGTH_M + GAPLENGTH_M)));
// drawPolygon draws centered lines -> must add half linewidth
// to height,width parameters of drawRect
                    int yTopOn = ystart - (int) ((lineWidth + 1) / 2);
                    //System.out.println("SimCanvas.paintLines:"
// + "xLeft="+xLeft);
                    g.fillRect(xLeft, yTopOn, lineLength + 1, lineWidth + 1);
                }
            }
        }


    }

    public void valuesIntersection(int choice_Szen, int priorFM, int priorFT, double qInFM, double qInFM1, double qInFM2, double qInFT, double qInFV,
            double perTrFM, double perTrFT, double perTrFV, double v0_limit,
            double p_factor, double deltaB, int tsleep_ms) {

        this.choice_Szen = choice_Szen;
        this.priorFM = priorFM;
        this.priorFT = priorFT;
        this.qInFM = qInFM;
        this.qInFM1 = qInFM1;
        this.qInFM2 = qInFM2;
        this.qInFT = qInFT;
        this.qInFV = qInFV;
        this.perTrFM = perTrFM;
        this.perTrFT = perTrFT;
        this.perTrFV = perTrFV;

        this.v0_limit = v0_limit;
        this.p_factor = p_factor;
        this.deltaB = deltaB;
        this.tsleep_ms = tsleep_ms;
        choice_Geom = 1;

        setScales();

        rd_Merida_m = STRAIGHT_FROMMERIDA_M;
        rd_Valle_m = STRAIGHT_FROMTOVALLE_M;
        straight1_m = STRAIGHT1_M;
        straight2_m = STRAIGHT2_M;
        straight3_m = STRAIGHT3_M;
        rd_ToTabay_m = straight3_m + Math.PI * RADIUS_M + straight2_m + Math.PI * RADIUS_M + straight1_m;

        if (choice_Szen == 8 || choice_Szen == 10) {
            rd_Valle_m = STRAIGHT_FROMTOVALLE_M + rd_Merida_m + CRUCE;
        }


        //setScales();


    }

    //valores para el caso de escenario 1,3,4 y 5
    public void newValues(int choice_Szen,
            double density, double qIn, double perTr,
            double v0_limit, double p_factor, double deltaB,
            int tsleep_ms) {


        this.choice_Szen = choice_Szen;
        this.density = density;
        this.qIn = qIn;
        this.perTr = perTr;
        this.v0_limit = v0_limit;
        this.p_factor = p_factor;
        this.deltaB = deltaB;
        this.tsleep_ms = tsleep_ms;

        choice_Geom = 1;

        setScales(); // NOT after update of rdlen, but behind choice_Szen

        rd_Merida_m = STRAIGHT_FROMMERIDA_M;
        rd_Valle_m = STRAIGHT_FROMTOVALLE_M;
        straight1_m = STRAIGHT1_M;
        straight2_m = STRAIGHT2_M;
        straight3_m = STRAIGHT3_M;
        rd_ToTabay_m = straight3_m + Math.PI * RADIUS_M + straight2_m + Math.PI * RADIUS_M + straight1_m;

        if (choice_Szen == 8 || choice_Szen == 10) {
            rd_Valle_m = STRAIGHT_FROMTOVALLE_M + rd_Merida_m;
        }



    }

    // valores para el caso de escenario 2
    public void newValues2(double qIn, double perTr,
            double p_factor, double deltaB,
            double qRamp, double p_factorRamp,
            double deltaBRamp, int tsleep_ms) {


        this.choice_Szen = 2;
        //setScales(); // NOT after update of rdlen, but behind choice_Szen
        this.qIn = qIn;
        this.perTr = perTr;
        this.p_factor = p_factor;
        this.qRamp = qRamp;
        this.p_factorRamp = p_factorRamp;
        this.deltaB = deltaB;
        this.deltaBRamp = deltaBRamp;
        this.tsleep_ms = tsleep_ms;
        choice_Geom = (STRAIGHT_ROAD) ? 2 : 1;
        rdlen_m = (choice_Geom == 2) // si la geometria es forma de Recta, 800m si no la mitad del circulo mas 2 rectas
                ? STRAIGHT_RDLEN_M
                : Math.PI * RADIUS_M + 2 * l_straight_m;
        System.out.println("SimCanvas.newValues2():"
                + " STRAIGHT_ROAD=" + STRAIGHT_ROAD
                + "rdlen_m = " + rdlen_m
                + " choice_Szen= " + choice_Szen
                + " choice_Geom= " + choice_Geom);
    }

    //martin jan05
    // change externally IDM parameters ONLY for cars
    public void changeIDMCarparameters(IDM idm) {
        System.out.println("in SimCanvas.changeIDMCarparameters");
        ((IDM) (microstreet.idmCar)).set_params(idm);
    }

    // martin oct07: changeIDMTruckparameters noch DOS wg
    // temp_car.setModel(..) in MicroStreet.java
    public void changeIDMTruckparameters(IDM idm) {
        System.out.println("in SimCanvas.changeIDMTruckparameters");
        ((IDM) (microstreet.idmTruck)).set_params(idm);
    }

    private void imposeSpeedLimit() {
        v0_limit_old = v0_limit;

        double v0trucks = (v0_limit < VMAX_TRUCK_KMH / 3.6)
                ? v0_limit
                : VMAX_TRUCK_KMH / 3.6;

        if ((choice_Szen == 3) || (choice_Szen == 5)) {
            ((IDM) (microstreet.idmCar)).set_v0(v0_limit);
            ((IDM) (microstreet.idmCar)).initialize();
            // comment following 2 lines if trucks are "free"
            ((IDM) (microstreet.idmTruck)).set_v0(v0trucks);
            ((IDM) (microstreet.idmTruck)).initialize();
        }
        if (choice_Szen == 4) {
            ((IDM) (microstreet.sync1Car)).set_v0(v0_limit);
            ((IDM) (microstreet.sync1Car)).initialize();
            // comment following 2 lines if trucks are "free"
            ((IDM) (microstreet.sync1Truck)).set_v0(v0trucks);
            ((IDM) (microstreet.sync1Truck)).initialize();
        }


    }

    // paint "Car" and "Truck" symbols and labels
    private void paintLegend(Graphics g) {
        /*
         double xRelRight = 0.98; // right end of the area for the legend
         double yRelSpaceBot = 0.02; // space between U road and bottom of area
         if (choice_Szen == 5) {
         yRelSpaceBot = 0.10;
         }
         // boolean withProbeCar=((choice_Szen==5)||(choice_Szen==1));
         boolean withProbeCar = false;
         String carString = lang.getCarString();
         String truckString = lang.getTruckString();
         String idString = lang.getIdString();
         String probeString = lang.getProbeVehString();
        
         int lineHeight = (int) (1.25 * metricsText.stringWidth("M"));
         int xoff = (withProbeCar)
         ? metricsText.stringWidth(probeString) + pkwLen
         : metricsText.stringWidth(truckString) + pkwLen;
         if (choice_Szen == 6) {
         xoff = metricsText.stringWidth(idString) + pkwLen;
         }
        
         int xmda = (int) (xRelRight * xsize) - xoff - lkwLen; // xsize,ysize in pixels
         int nUp = (withProbeCar) ? 2 : 1;
         int y0 = yc + r1 - (int) (yRelSpaceBot * ysize) - nUp * lineHeight;
         //System.out.println("y0="+y0);
         if (choice_Szen == 6) {
         y0 += lineHeight;
         }
         //if(choice_Geom==2){ y0=ystart - 4*lineHeight;}
        
        
         // text "Cars" or "Trucks"
        
         g.setFont(textFont);
         g.setColor(Color.black);
        
         g.drawString(carString, xmda, y0);
         g.drawString((choice_Szen == 6) ? idString : truckString, xmda, y0 + lineHeight);
         if (withProbeCar) {
         g.drawString(probeString, xmda, y0 + 2 * lineHeight);
         }
        
         // vehicle symbols
        
         g.setColor(MicroStreet.colorCar);
         g.fillRect(xmda + xoff, y0 - vehWidth, pkwLen, vehWidth);
         if (choice_Szen == 6) {
         g.setColor(Color.green);
         g.fillRect(xmda + xoff, y0 + lineHeight - vehWidth, pkwLen, vehWidth);
         } else {
         g.setColor(MicroStreet.colorTruck);
         g.fillRect(xmda + xoff, y0 + lineHeight - vehWidth, lkwLen, vehWidth);
         }
         if (withProbeCar) {
         g.setColor(Color.blue);
         g.fillRect(xmda + xoff, y0 + 2 * lineHeight - vehWidth, pkwLen, vehWidth);
         }*/
    }

    /*
     // alte PaintLegend. Loeschen, wenn die neue "zeitgetestet"
     private void paintLegend(Graphics g){
    
     // boolean withProbeCar=((choice_Szen==5)||(choice_Szen==1));
     boolean withProbeCar=false;
     String carString = (GERMAN) ? "PKW" : "Car";
     String truckString = (GERMAN) ? "LKW" : "Truck";
     String idString = (GERMAN) ? "Gleicher PKW-Typ" : "Same Car";
     String probeString = (GERMAN) ? "Testfahrzeug" : "Probe Car";
    
     int lineHeight=(int)(1.25*metricsText.stringWidth("M"));
     int xmda=(int)(0.025*xsize); // xsize,ysize in pixels
     int y0=yc+r2-3*lineHeight;
     if (choice_Szen==6){y0+=lineHeight;}
     if(choice_Geom==2){ y0=ystart - 4*lineHeight;}
    
     int xoff=(withProbeCar)
     ? metricsText.stringWidth(probeString)+pkwLen
     : metricsText.stringWidth(truckString)+pkwLen;
     if (choice_Szen==6){
     xoff=metricsText.stringWidth(idString)+pkwLen;}
    
     // text "Cars" or "Trucks"
    
     g.setFont(textFont);
     g.setColor(Color.black);
    
     g.drawString(carString,xmda,y0);
     g.drawString( (choice_Szen==6)?idString:truckString, xmda,y0+lineHeight);
     if (withProbeCar){
     g.drawString(probeString,xmda, y0+2*lineHeight);
     }
    
     // vehicle symbols
    
     g.setColor(Color.red);
     g.fillRect (xmda+xoff, y0 -vehWidth, pkwLen, vehWidth);
     if (choice_Szen==6){
     g.setColor(Color.green);
     g.fillRect (xmda+xoff, y0+lineHeight -vehWidth, pkwLen, vehWidth);}
     else{
     g.setColor(Color.black);
     g.fillRect (xmda+xoff, y0+lineHeight -vehWidth, lkwLen, vehWidth);}
     if (withProbeCar){
     g.setColor(Color.blue);
     g.fillRect(xmda+xoff, y0+2*lineHeight-vehWidth, pkwLen, vehWidth);
     }
     }
     */
    private void paintTimes(Graphics g) {

        int min, sec;
        int xmda = 10; // start position of time display (left upper corner)
        int y0 = 20;

        min = (int) (time / 60.0);
        sec = (int) (time - min * 60.0);
        String str_time = lang.getSimTime();
        String timeString = str_time
                + (new Integer(min)).toString()
                + ((sec > 9) ? ":" : ":0") + (new Integer(sec)).toString();
        int widthTime = metricsText.stringWidth(str_time);
        int widthDisplay = metricsText.stringWidth(timeString);

        // clear old time

        g.setColor(backgrColor);
        // g.setColor(Color.white);
        g.fillRect(xmda + widthTime, y0 - textHeight, widthDisplay - widthTime, textHeight);

        // display new time

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(timeString, xmda, y0);

        // draw "Countdown <m>:<ss>" at right bottom pos
/*
         if (choice_Szen == 5) {
         if (time < T_GREEN) {
         min = (int) ((T_GREEN - time) / 60.0);
         sec = (int) ((T_GREEN - time) - min * 60.0);
         String countdString = "Countdown " + (new Integer(min)).toString()
         + ((sec > 9) ? ":" : ":0") + (new Integer(sec)).toString();
        
         widthTime = metricsText.stringWidth("Countdown");
         widthDisplay = metricsText.stringWidth(countdString);
         xmda = l_straight + xc - 5 - widthDisplay;
         y0 = yc + r2 - lw - 20;
        
         //g.setColor(backgrColor);
         g.setColor(Color.white);
         g.fillRect(xmda + widthTime, y0 - textHeight,
         widthDisplay - widthTime, textHeight);
         g.setColor(Color.black);
         g.drawString(countdString, xmda, y0);
         }
         }*/
    }

    //pinta el tiempo de servicio
    private void paintResultados(Graphics g) {

        //COLAS
        int x0 = 600 * (int) scale; // start position
        int y0 = ystart + 16 * lw;

        //String countdString = "Service Time From Tabay ";
        int nVehFM = fromMerida.street.size();
        int nVehFT = fromTabay.street.size();
        int nVehFV = fromValle.street.size() - 1;
        int nVehTM = toMerida.street.size();
        int nVehTT = toTabay.street.size() - 1;
        int nVehTV = toValle.street.size();

        g.setColor(Color.WHITE);
        g.fillRect(x0, y0 - lw, 15 * lw, 9 * lw);
        g.setColor(Color.GRAY);
        g.fillRect(x0, y0 - lw, 15 * lw, 2 * lw);
        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(" Numero de vehiculos en cada canal", x0, y0);


        y0 = y0 + 2 * lw;
        //From Merida
        String str_cola = " Desde Merida:     ";
        //String colaString = "";
        if (choice_Szen == 9 || choice_Szen == 10) {
            nVehFM = nVehFM-1;
            int nVehFM2 = fromMerida2.street.size()-1;
            int acum = nVehFM + nVehFM2;
            String colaString = str_cola + (new Integer(acum));
            int widthCola = metricsText.stringWidth(str_cola);
            int widthDisplay = metricsText.stringWidth(colaString);
            //Clear old nveh
            g.setColor(Color.WHITE);
            g.fillRect(x0 + widthCola, y0 - textHeight, widthDisplay - widthCola, textHeight);

            // display new nveh

            g.setFont(textFont);
            g.setColor(Color.black);
            g.drawString(colaString, x0, y0);
        } else {
            String colaString = str_cola + (new Integer(nVehFM - 1));
            int widthCola = metricsText.stringWidth(str_cola);
            int widthDisplay = metricsText.stringWidth(colaString);
            //Clear old nveh
            g.setColor(Color.WHITE);
            g.fillRect(x0 + widthCola, y0 - textHeight, widthDisplay - widthCola, textHeight);

            // display new nveh

            g.setFont(textFont);
            g.setColor(Color.black);
            g.drawString(colaString, x0, y0);
        }


        //From Tabay

        y0 = y0 + lw;
        str_cola = " Desde Tabay:       ";
        String colaString = str_cola + (new Integer(nVehFT - 1));
        int widthCola = metricsText.stringWidth(str_cola);
        int widthDisplay = metricsText.stringWidth(colaString);
        //Clear old nveh
        g.setColor(Color.WHITE);
        g.fillRect(x0 + widthCola, y0 - textHeight, widthDisplay - widthCola, textHeight);

        // display new nveh

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(colaString, x0, y0);

        //From Valle

        y0 = y0 + lw;
        str_cola = " Desde El Valle:    ";
        colaString = str_cola + (new Integer(nVehFV));
        widthCola = metricsText.stringWidth(str_cola);
        widthDisplay = metricsText.stringWidth(colaString);
        //Clear old nveh
        g.setColor(Color.WHITE);
        g.fillRect(x0 + widthCola, y0 - textHeight, widthDisplay - widthCola, textHeight);

        // display new nveh

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(colaString, x0, y0);

        //To Merida

        y0 = y0 + lw;
        str_cola = " Hacia Merida:       ";
        colaString = str_cola + (new Integer(nVehTM));
        widthCola = metricsText.stringWidth(str_cola);
        widthDisplay = metricsText.stringWidth(colaString);
        //Clear old nveh
        g.setColor(Color.WHITE);
        g.fillRect(x0 + widthCola, y0 - textHeight, widthDisplay - widthCola, textHeight);

        // display new nveh

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(colaString, x0, y0);

        //To Tabay

        y0 = y0 + lw;
        str_cola = " Hacia Tabay:        ";
        colaString = str_cola + (new Integer(nVehTT));
        widthCola = metricsText.stringWidth(str_cola);
        widthDisplay = metricsText.stringWidth(colaString);
        //Clear old nveh
        g.setColor(Color.WHITE);
        g.fillRect(x0 + widthCola, y0 - textHeight, widthDisplay - widthCola, textHeight);

        // display new nveh

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(colaString, x0, y0);

        //Hacia El Valle:

        y0 = y0 + lw;
        str_cola = " Hacia El Valle:      ";
        colaString = str_cola + (new Integer(nVehTV));
        widthCola = metricsText.stringWidth(str_cola);
        widthDisplay = metricsText.stringWidth(colaString);
        //Clear old nveh
        g.setColor(Color.WHITE);
        g.fillRect(x0 + widthCola, y0 - textHeight, widthDisplay - widthCola, textHeight);

        // display new nveh

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(colaString, x0, y0);

//TIEMPOS DE SERVICIO

        x0 = 800 * (int) scale;
        y0 = ystart + 16 * lw;

        g.setColor(Color.WHITE);
        g.fillRect(x0, y0 - lw, 16 * lw, 7 * lw);

        g.setColor(Color.GRAY);
        g.fillRect(x0, y0 - lw, 16 * lw, 2 * lw);
        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString("     Tiempo promedio de servicio ", x0, y0);


        //Desde Tabay hacia Merida
        y0 = y0 + 2 * lw;
        double sTime = toMerida.serviceTFT;
        //int numVehFT =  toMerida.numVehFT;        
        //double sumTSFT =  toMerida.sumTimesFT;
        
        String str_lane = " Desde Tabay hacia Merida:    ";
        String str_sTime = " seg ";
        String serviceTime = str_lane + (new Integer((int) sTime)) + str_sTime;
        double widthserviceT = metricsText.stringWidth(str_sTime);
        widthDisplay = metricsText.stringWidth(serviceTime);

        //Clear old service time
        g.setColor(Color.WHITE);
        g.fillRect(x0 + (int) widthserviceT, y0 - textHeight, widthDisplay - (int) widthserviceT, textHeight);

        // display new service time

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(serviceTime, x0, y0);

        //Desde El Valle hacia Merida
        y0 = y0 + lw;
        //int numVehFV =  toMerida.numVehFV;
        //double sumTSFV =  toMerida.sumTimesFV;
        
        if(choice_Szen == 8 || choice_Szen == 10)
            sTime = fromValle.serviceTFV;
        else{
            sTime = toMerida.serviceTFV;
            
        }
        
        str_lane = " Desde El Valle hacia Merida:  ";
        str_sTime = " seg ";
        
        serviceTime = str_lane + (new Integer((int) sTime)) + str_sTime;
        widthserviceT = metricsText.stringWidth(str_sTime);
        widthDisplay = metricsText.stringWidth(serviceTime);

        //Clear old service time
        g.setColor(Color.WHITE);
        g.fillRect(x0 + (int) widthserviceT, y0 - textHeight, widthDisplay - (int) widthserviceT, textHeight);

        // display new service time

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(serviceTime, x0, y0);


        //Desde Merida hacia El Valle

        y0 = y0 + lw;
        sTime = toValle.serviceTFMTV;

        str_lane = " Desde Merida hacia El Valle:  ";
        str_sTime = " seg ";
        serviceTime = str_lane + (new Integer((int) sTime)) + str_sTime;
        widthserviceT = metricsText.stringWidth(str_sTime);
        widthDisplay = metricsText.stringWidth(serviceTime);

        //Clear old service time
        g.setColor(Color.WHITE);
        g.fillRect(x0 + (int) widthserviceT, y0 - textHeight, widthDisplay - (int) widthserviceT, textHeight);

        // display new service time

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(serviceTime, x0, y0);

        // Desde Merida hacia Tabay

        y0 = y0 + lw;
        sTime = toTabay.serviceTFMTT;

        str_lane = " Desde Merida hacia Tabay:    ";
        str_sTime = " seg ";
        serviceTime = str_lane + (new Integer((int) sTime)) + str_sTime;
        widthserviceT = metricsText.stringWidth(str_sTime);
        widthDisplay = metricsText.stringWidth(serviceTime);

        //Clear old service time
        g.setColor(Color.WHITE);
        g.fillRect(x0 + (int) widthserviceT, y0 - textHeight, widthDisplay - (int) widthserviceT, textHeight);

        // display new service time

        g.setFont(textFont);
        g.setColor(Color.black);
        g.drawString(serviceTime, x0, y0);
    }

    private void paintSymbols(Graphics g) {

        // traffic light
        // t<T_GREEN: red; otherwise green

        if (choice_Szen == 5) {
            double det1pos = RELPOS_TL * rdlen_m;
            int detx = (new DetPosU(det1pos)).x;
            int dety = (new DetPosU(det1pos)).y - lw;
            int heightTL = (int) (radius * 0.16); // total height of T.L. // radius = (int)(RADIUS_M*scale)
            int radiusTL = (int) (radius * 0.024); // radius of lights
            g.fillRect(detx, dety - heightTL, 2, heightTL); // pole
            g.fillRect(detx - radiusTL, dety - heightTL,
                    2 * radiusTL, 4 * radiusTL);

            if (time < T_GREEN) {
                g.setColor(Color.red);
                g.fillOval(detx - radiusTL, dety - heightTL,
                        2 * radiusTL, 2 * radiusTL);
            } else {
                g.setColor(Color.green);
                g.fillOval(detx - radiusTL, dety - heightTL + 2 * radiusTL,
                        2 * radiusTL, 2 * radiusTL);
            }
        }

        // paint the "virtual traffic light" controlling too much inflow

        if ((choice_Szen == 5) || (choice_Szen == 3) || (choice_Szen == 4)) {

            if (microstreet.red == true) {
                g.setColor(Color.black);
                int bw = (int) (radius * 0.1);
//                g.fillRect((l_straight + xc - 5 - bw), yc - r1 + 5, bw, bw);
                g.setColor(Color.red);
                //              g.fillOval((l_straight + xc - 5 - bw), yc - r1 + 5, bw, bw);
            } else {
                g.setColor(Color.black);
                int bw = (int) (radius * 0.1);
                //            g.fillRect((l_straight + xc - 5 - bw), yc - r1 + 5, bw, bw);
                g.setColor(Color.green);
                //          g.fillOval((l_straight + xc - 5 - bw), yc - r1 + 5, bw, bw);
            }
        }




        // paint Uphill traffic Sign (US)
        // for the flow-conserving bottleneck Scen.

        if (choice_Szen == 4) {
            double detpos = l_straight_m + 0.5 * Math.PI * RADIUS_M;

// positin of pole
            int heightTot = (int) (radius * 0.24); // total height of U.S. // (int)(RADIUS_M*scale)
            // scalex = xsize/xsize_m;
            // scaley = ysize/ysize_m;
            int heightSign = (int) (radius * 0.16);
            int xmda = (new DetPosU(detpos)).x + (int) (0.2 * radius);
            int y0 = (new DetPosU(detpos)).y + heightTot;
            int xc = xmda;
            int yc = y0 - heightTot + (int) (0.5 * heightSign);
            double relWidthRed = 0.2; //


// draw pole
            g.setColor(Color.black);
            g.fillRect(xmda, y0 - heightTot, 2, heightTot);

// draw triangular sign
            int hbasis = (int) (0.577 * heightSign); // tan(30 deg)
            int vbasis = (int) (0.5 * heightSign);
            int[] ptsx = {xc, xc - hbasis, xc + hbasis};
            int[] ptsy = {yc - vbasis, yc + vbasis, yc + vbasis};
            g.setColor(Color.red);
            g.fillPolygon(ptsx, ptsy, ptsx.length);

            yc += (int) (0.3 * relWidthRed * vbasis);
            hbasis = (int) ((1 - relWidthRed) * hbasis);
            vbasis = (int) ((1 - relWidthRed) * vbasis);
            int[] ptsx1 = {xc, xc - hbasis, xc + hbasis};
            int[] ptsy1 = {yc - vbasis, yc + vbasis, yc + vbasis};
            g.setColor(Color.white);
            g.fillPolygon(ptsx1, ptsy1, ptsx1.length);

// draw gradient
            ptsx1[0] = (int) (0.5 * (ptsx[0] + ptsx[2]));
            ptsy1[0] = (int) (0.5 * (ptsy[0] + ptsy[2]));

            g.setColor(Color.black);
            g.fillPolygon(ptsx1, ptsy1, ptsx1.length);

// text

            y0 += heightSign;
            g.drawString(lang.getUphillMarkerBegin(), xmda, y0);
            y0 += (int) (0.07 * radius);
            g.drawString(lang.getUmhillMarker(), xmda, y0);

        }


        // paint Optional text "Engstelle" and crop marks

        if (false) {
            int xmda = (int) (0.25 * xsize);
            int y0 = (int) (0.82 * ysize);
            if (choice_Szen == 3) { // choice_Szen==3: lane closing
                g.setFont(textFont);
                g.setColor(Color.black);
                g.drawString("Engstelle", xmda, y0);
            }

            int x1mark = (int) (0.10 * xsize);
            int y1mark = (int) (0.50 * ysize);
            int x2mark = (int) (0.70 * xsize);
            int y2mark = (int) (0.90 * ysize);
            int markLen = (int) (0.05 * xsize);

            if (choice_Szen == 3) { // choice_Szen==3: lane closing
                // g.setFont(textFont);
                g.setColor(Color.black);
                g.drawLine(x1mark, y1mark, x1mark + markLen, y1mark);
                g.drawLine(x2mark, y2mark, x2mark, y2mark - markLen);
            }
        }
    }

    // paint the speed-limit sign
    private void paintSpeedLimit(Graphics g) {
        int v0_kmh = (int) (3.6 * v0_limit);
        int heightSS = (int) (radius * 0.24); // total height of T.L.
        int radiusSS = (int) (radius * 0.08); // radius of lights
        double det1pos = RELPOS_SPEEDSIGN * rdlen_m;
        int xmda = (new DetPosU(det1pos)).x; // basis of pole, base del poste
        int y0 = (new DetPosU(det1pos)).y + (int) (1.5 * lw) + heightSS;
        int xc = xmda; // center of circular sign
        int yc = y0 - heightSS + radiusSS; // center of circular sign

        g.setColor(Color.black);
        g.fillRect(xmda, y0 - heightSS, 2, heightSS); // pole

        if (v0_kmh >= V0_LIMIT_MAX_KMH) { // no speed limit
            g.setColor(Color.white);
            g.fillOval(xc - radiusSS, yc - radiusSS, 2 * radiusSS, 2 * radiusSS);
            g.setColor(Color.black);
            int space = (int) (0.15 * radiusSS);
            for (int i = -2; i <= 2; i++) {
                g.drawLine(xc - (int) (0.7 * radiusSS) + i * space,
                        yc + (int) (0.7 * radiusSS) + i * space,
                        xc + (int) (0.7 * radiusSS) + i * space,
                        yc - (int) (0.7 * radiusSS) + i * space);
            }
        } else { // speed limit active
            int rwhite = (int) (0.8 * radiusSS);
            String speedLimitStr = String.valueOf(v0_kmh);
            int ref_width = metricsText.stringWidth(speedLimitStr);
            int speedFontHeight = (int) (textHeight * 1.4 * radiusSS / ref_width);
            Font speedFont = new Font("SansSerif", Font.BOLD, speedFontHeight);
            int xoff = (int) (0.65 * radiusSS);
            int yoff = (int) (0.4 * speedFontHeight);

            g.setColor(Color.red);
            g.fillOval(xc - radiusSS, yc - radiusSS, 2 * radiusSS, 2 * radiusSS);
            g.setColor(Color.white);
            g.fillOval(xc - rwhite, yc - rwhite, 2 * rwhite, 2 * rwhite);
            g.setColor(Color.black);
            g.setFont(speedFont);
            g.drawString(String.valueOf(v0_kmh), xc - xoff, yc + yoff);
            g.setFont(textFont);
        }
    }

    // lane=0: left/inner
    // lane=1: right/outer
    // lane=-1: polygon for road lines
    // lane double-valued for continuous transition!
    class PolyVehCircle {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public PolyVehCircle(double pos, double lane,
                double length, double width, int r1, int r2, int xc, int yc) {

            double l = length * scale; // length of drawn car
            double w = width * scale; // width of drawn car

            double center_pos = pos + length / 2; // in sim pos=back(!) end
            double arc = -center_pos / RADIUS_M - 0.25 * (Math.PI);
            double ca = Math.cos(arc);
            double sa = ((CLOCKWISE) ? -1 : 1) * Math.sin(arc);
            double r = (lane >= 0) ? r1 + (0.25 + 0.5 * lane) * (r2 - r1)
                    : r1 + 0.5 * (r2 - r1);

// pos=7/8 * CIRCUMF_M -> arc=2 pi -> arc=0 -> center at (xc+r,yc)

            double xc_car = (xc + r * ca);
            double yc_car = (yc + r * sa); // graph y = - math y

            int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
            int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
            int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
            int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

            int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
            int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
            int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
            int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));


            xPoints[0] = x_frontright;
            xPoints[1] = x_frontleft;
            xPoints[2] = x_backleft;
            xPoints[3] = x_backright;

            yPoints[0] = y_frontright;
            yPoints[1] = y_frontleft;
            yPoints[2] = y_backleft;
            yPoints[3] = y_backright;

// Polygon instead explicit arrays: 15 % runtime perf. loss!
// vehShape = new Polygon(xPoints, car_yPoints, 4);
        }
    }

    // lane=0: left/inner
    // lane=1: right/outer
    // lane=-1: polygon for road lines
    // lane double-valued for continuous transition!
    class PolyVehU {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public PolyVehU(double pos, double lane, double length, double width,
                int r1, int r2, int xc, int l_straight, int yc, int rdlen) {

            double l = length * scale; // length of drawn car
            double w = width * scale; // width of drawn car
            double center_pos = pos + length / 2; // in sim pos=back (!) end
            int intpos = (int) (center_pos * scale);
            double r = (lane >= 0) ? r1 + (0.25 + 0.5 * lane) * (r2 - r1)
                    : r1 + 0.5 * (r2 - r1);

            if ((intpos > l_straight) && (intpos < (rdlen - l_straight))) {// curva

                double arc = -0.5 * (Math.PI)
                        - (2 * (Math.PI) * ((double) (intpos - l_straight)))
                        / ((double) (2 * (rdlen - 2 * l_straight)));

                double ca = Math.cos(arc);
                double sa = Math.sin(arc);
                double xc_car = (xc + r * ca);
                double yc_car = (yc + r * sa);

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }
            if (intpos <= l_straight) {//primera recta

                double ca = 0.0;
                double sa = 1.0;

                double xc_car = xc + l_straight - intpos;
                double yc_car = yc - r;

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }

            if (intpos > (rdlen - l_straight)) {//segunda recta

                double ca = 0.0;
                double sa = -1.0;

                double xc_car = xc + l_straight + intpos - rdlen;
                double yc_car = yc + r;

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));


                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }
        }
    }
    // POLIGONOS PARA LA RUTA DESDE TABAY

    class PolyVehStraight {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public PolyVehStraight(double pos_m, double lane,
                double vehLen_m, double vehWidth_m, int rdlen) {

            double l = vehLen_m * scale; // length of drawn car
            double w = vehWidth_m * scale; // width of drawn car
            int xveh = xstart + (int) ((pos_m + vehLen_m / 2) * scale);
            double yveh = (lane >= 0) ? ystart - (1.5 - lane) * lw
                    : ystart - lw;

            if (true) {


                int x_frontleft = (int) (xveh - 0.5 * l);
                int x_frontright = (int) (xveh - 0.5 * l);
                int x_backleft = (int) (xveh + 0.5 * l);
                int x_backright = (int) (xveh + 0.5 * l);

                int y_frontleft = (int) (yveh + 0.5 * w);
                int y_frontright = (int) (yveh - 0.5 * w);
                int y_backleft = (int) (yveh + 0.5 * w);
                int y_backright = (int) (yveh - 0.5 * w);

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }
        } // end constructor
    } // end PolyVehStraight

    class polyVehFromTabay {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public polyVehFromTabay(double pos, double length, double width, int r1, int r2,
                int xc1, int yc1, int xc2, int yc2,
                int straight1, int straight2, int straight3, int rdlen) {

            double l = length * scale; // length of drawn car
            double w = width * scale; // width of drawn car
            double center_pos = pos + length / 2; // in sim pos=back (!) end
            int intpos = (int) (center_pos * scale);

            // curva 2
            if ((intpos > straight3) && (intpos < (rdlen - (straight2 + Math.PI * (r2 - lw)) + straight1))) {
                double r = r1 + 0.25 * (r2 - r1);// =15
                double arc = -0.5 * (Math.PI)
                        - (2 * (Math.PI) * ((double) (intpos - straight3)))
                        / ((double) (2 * (rdlen - (straight3 + straight2 + straight1 + Math.PI * (r2 - lw)))));

                double ca = Math.cos(arc);
                double sa = -Math.sin(arc);
                double xc_car = (xc2 + r * ca);
                double yc_car = (yc2 + 2 * r + r * sa);

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }
            //recta 3: desde Rio Mucujun hasta inicio de curva 2
            if (intpos <= straight3) {

                //System.out.println(" Recta 3 de FromMerida intpos "+ intpos);
                double r = r1 + 0.25 * (r2 - r1);// =15
                double ca = 0.0;
                double sa = 1.0;

                double xc_car = xc2 + straight3 - intpos;
                double yc_car = yc2 + 2 * r + r;

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }

            // recta2: desde fin de curva 2 hasta inicio de curva 1
            if ((intpos > (straight3 + Math.PI * (r2 - lw))) && (intpos < rdlen - (straight1 + Math.PI * (r2 - lw)))) {
                double r = r1 + 0.25 * (r2 - r1);// =15
                double ca = 0.0;
                double sa = -1.0;

                //double xc_car = xc2 + r;// + intpos - rdlen;
                double xc_car = -(straight2 - r2) + intpos;// - rdlen;//2 * xc1 - 2 * r1 - r + intpos - rdlen;
                double yc_car = yc2 + r;

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }
            // curva1
            if ((intpos > (straight3 + Math.PI * (r2 - lw) + straight2)) && (intpos < (rdlen - straight1))) {
                double r = r1 + 0.75 * (r2 - r1);// =25
                double arc = -0.5 * (Math.PI)
                        - (2 * (Math.PI) * ((double) (intpos - (straight3 + Math.PI * (r2 - lw) + straight2))))
                        / ((double) (2 * (rdlen - (straight1 + straight2 + straight3 + Math.PI * (r2 - lw)))));

                double ca = -Math.cos(arc);
                double sa = -Math.sin(arc);
                double xc_car = (xc1 + r * ca);
                double yc_car = (yc1 + r2 + r * sa);

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }
            // recta1: desde inicio de curva 1 hasta la interseccion      
            if (intpos >= (straight3 + Math.PI * (r2 - lw) + straight2 + Math.PI * (r2 - lw))) {

                double r = r1 + 0.75 * (r2 - r1);// =25
                double ca = 0.0;
                double sa = 1.0;

                double xc_car = xc1 + (straight3 + Math.PI * (r2 - lw) + straight2 + Math.PI * (r2 - lw)) - intpos;
                double yc_car = yc1 - lw - lw + r;

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }
        }
    }

    // POLIGONOS PARA LA RUTA HACIA TABAY
    class polyVehToTabay {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public polyVehToTabay(double pos, double length, double width, int r1, int r2,
                int xc1, int yc1, int xc2, int yc2,
                int straight1, int straight2, int straight3, int rdlen) {

            double l = length * scale; // length of drawn car
            double w = width * scale; // width of drawn car
            double center_pos = pos + length / 2; // in sim pos=back (!) end
            int intpos = (int) (center_pos * scale);
            int cruce = (int) (CRUCE * scale);
            int radio = (int) (scale * RADIUS_M);
            //para pintar los vehiculos en la interseccion de Merida hacia Tabay 

            if (choice_Szen == 7 || choice_Szen == 8) {
                if (intpos < cruce) {
                    double sa = Math.sin(-ANGLE_ACCESS);
                    double ca = Math.cos(ANGLE_ACCESS);
                    double xs_m = 0.; // start merging
                    double ys_m = ystart / scale; // start merging
                    double xveh = scale * STRAIGHT_FROMMERIDA_M + scale * (xs_m + ca * (pos + 0.25 * length));
                    double yveh = 1.5 * lw + scale * (ys_m - sa * (pos + 0.25 * length));


                    int x_frontleft = (int) (xveh + 0.5 * (ca * l - sa * w));
                    int x_frontright = (int) (xveh + 0.5 * (ca * l + sa * w));
                    int x_backleft = (int) (xveh + 0.5 * (-ca * l - sa * w));
                    int x_backright = (int) (xveh + 0.5 * (-ca * l + sa * w));

                    int y_frontleft = (int) (yveh + 0.5 * (-sa * l - ca * w));
                    int y_frontright = (int) (yveh + 0.5 * (-sa * l + ca * w));
                    int y_backleft = (int) (yveh + 0.5 * (sa * l - ca * w));
                    int y_backright = (int) (yveh + 0.5 * (sa * l + ca * w));

                    xPoints[0] = x_frontright;
                    xPoints[1] = x_frontleft;
                    xPoints[2] = x_backleft;
                    xPoints[3] = x_backright;
                    xPoints[4] = x_frontright;

                    yPoints[0] = y_frontright;
                    yPoints[1] = y_frontleft;
                    yPoints[2] = y_backleft;
                    yPoints[3] = y_backright;
                    yPoints[4] = y_frontright;
                }
                if (intpos >= cruce && intpos < (straight1 + cruce)) {

                    double r = r1 + 0.25 * (r2 - r1);// =15
                    double ca = 0.0;
                    double sa = -1.0;

                    double xc_car = rd_Merida + intpos;
                    double yc_car = yc1 + r;

                    int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                    int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                    int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                    int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                    int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                    int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                    int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                    int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                    xPoints[0] = x_frontright;
                    xPoints[1] = x_frontleft;
                    xPoints[2] = x_backleft;
                    xPoints[3] = x_backright;
                    xPoints[4] = x_frontright;

                    yPoints[0] = y_frontright;
                    yPoints[1] = y_frontleft;
                    yPoints[2] = y_backleft;
                    yPoints[3] = y_backright;
                    yPoints[4] = y_frontright;
                }

            }
            // recta1: desde la interseccion hasta el inicio de la curva 1            
            if (choice_Szen == 9 || choice_Szen == 10) {
                if (intpos < (rdlen - (straight3 + Math.PI * (radio) + straight2 + Math.PI * (radio)))) {

                    double r = r1 + 0.25 * (r2 - r1);// =15
                    double ca = 0.0;
                    double sa = -1.0;

                    double xc_car = rd_Merida + intpos;
                    double yc_car = yc1 + r;

                    int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                    int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                    int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                    int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                    int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                    int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                    int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                    int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                    xPoints[0] = x_frontright;
                    xPoints[1] = x_frontleft;
                    xPoints[2] = x_backleft;
                    xPoints[3] = x_backright;
                    xPoints[4] = x_frontright;

                    yPoints[0] = y_frontright;
                    yPoints[1] = y_frontleft;
                    yPoints[2] = y_backleft;
                    yPoints[3] = y_backright;
                    yPoints[4] = y_frontright;
                }
            }

            // curva1
            if ((intpos > straight1 + cruce) && (intpos < (cruce + straight1 + Math.PI * (radio)))) {//(rdlen - (straight2 + (Math.PI * (r2-lw)) + straight3)
                double r = r1 + 0.25 * (r2 - r1);// =15
                double arc = -0.5 * (Math.PI)
                        - (2 * (Math.PI) * ((double) (intpos - (straight1 + cruce))))
                        / ((double) (2 * (rdlen - (cruce + straight1 + straight2 + straight3 + Math.PI * (radio)))));

                double ca = -Math.cos(arc);
                double sa = Math.sin(arc);
                double xc_car = (xc1 + r * ca);
                double yc_car = (yc1 + r2 + r * sa);

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }

            // recta2: desde fin de curva 1 hasta inicio de curva 2
            if ((intpos >= (cruce + straight1 + Math.PI * (radio))) && (intpos <= (rdlen - (Math.PI * radio + straight3)))) {
                double r = r1 + 0.25 * (r2 - r1);// =15
                double ca = 0.0;
                double sa = 1.0;


                double xc_car = rdlen + 1.5 * cruce - straight2 - intpos;//xc2 + 4 * straight2 - intpos;
                double yc_car = yc1 + 3 * r;                                                //xc2= 

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }

            // curva 2
            if ((intpos > (cruce + straight1 + Math.PI * (radio) + straight2)) && (intpos < (rdlen - (straight3)))) {
                double r = r1 + 0.75 * (r2 - r1);// =15
                double arc = -0.5 * (Math.PI)
                        - (2 * (Math.PI) * ((double) (intpos - (cruce + straight1 + Math.PI * (radio) + straight2))))
                        / ((double) (2 * (rdlen - (cruce + straight1 + Math.PI * (radio) + straight2 + straight3))));

                double ca = Math.cos(arc);
                double sa = Math.sin(arc);
                double xc_car = (xc2 + r * ca);
                double yc_car = (yc2 + lw + 2 * r1 + r * sa);

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }


            //recta 3: desde el fin de la curva 2 hasta el Rio Mucujun
            if (intpos >= rdlen - straight3) {
                double r = r1 + 0.75 * (r2 - r1);// =15
                double ca = 0.0;
                double sa = -1.0;

                double xc_car = xc2 + 2 * roadMargin + straight1 + intpos - rdlen;//intpos - xc2 + r1;
                //System.out.println(" cuerva 3 pos x "+ xc_car);
                yc2 = yc1 + 2 * r1 + 4 * (int) lw + r1 + (int) r;
                double yc_car = yc2;

                int x_frontleft = (int) (xc_car + 0.5 * (-w * ca - l * sa));
                int x_frontright = (int) (xc_car + 0.5 * (w * ca - l * sa));
                int x_backleft = (int) (xc_car + 0.5 * (-w * ca + l * sa));
                int x_backright = (int) (xc_car + 0.5 * (w * ca + l * sa));

                int y_frontleft = (int) (yc_car + 0.5 * (-w * sa + l * ca));
                int y_frontright = (int) (yc_car + 0.5 * (w * sa + l * ca));
                int y_backleft = (int) (yc_car + 0.5 * (-w * sa - l * ca));
                int y_backright = (int) (yc_car + 0.5 * (w * sa - l * ca));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }

        }
    }

    // lane=0: left/inner
    // lane=1: right/outer
    // lane=-1: polygon for road lines
    // lane double-valued for continuous transition!
    // pos = vehicle pos in m!
    // length,width: vehicle dimensions in m!
    // xstart, ystart: left beginning road (pixel units!)
    // rdlen: road length (pixel units!)
    //POLIGONOS FROM MERIDA
    class PolyVehFromMerida {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public PolyVehFromMerida(double pos_m,
                double vehLen_m, double vehWidth_m, int rdlen, double yveh) {

            double l = vehLen_m * scale; // length of drawn car
            double w = vehWidth_m * scale; // width of drawn car
            if (pos_m > 0.) { // si hay vehiculos en el canal
                int xveh = (int) ((pos_m + vehLen_m / 2) * scale);
                //double yveh = ystart + 1.5 * lw + lw;


                int x_frontleft = (int) (xveh + 0.5 * l);
                int x_frontright = (int) (xveh + 0.5 * l);
                int x_backleft = (int) (xveh - 0.5 * l);
                int x_backright = (int) (xveh - 0.5 * l);

                int y_frontleft = (int) (yveh - 0.5 * w);
                int y_frontright = (int) (yveh + 0.5 * w);
                int y_backleft = (int) (yveh - 0.5 * w);
                int y_backright = (int) (yveh + 0.5 * w);

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }

        } // end constructor
    } // end PolyVehFromMerida

    class PolyVehToMerida {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public PolyVehToMerida(double pos_m,
                double vehLen_m, double vehWidth_m, int rdlen) {

            double l = vehLen_m * scale; // length of drawn car
            double w = vehWidth_m * scale; // width of drawn car
            double sa = Math.sin(-ANGLE_ACCESSFM);
            double ca = Math.cos(-ANGLE_ACCESSFM);
            double region = CRUCE/2;//(Math.sqrt(Math.pow(ANCHO_CRUCE, 2) + Math.pow(LARGO_CRUCE, 2)));
            //double region = CRUCE;
            // Cambiar
            if (pos_m > region) { // actual merging section
                //int xstartTM = -(int) (scale * (pos_m - fromMerida.length())+10);
                int xveh = rdlen + (int) (scale * CRUCE) - (int) (scale * (pos_m + vehLen_m / 2)); //- (int)(scale*LARGO_CRUCE); //-(int) ((pos_m - vehLen_m / 2) * scale);
                double yveh = ystart + 0.5*lw;


                int x_frontleft = (int) (xveh + 0.5 * l);
                int x_frontright = (int) (xveh + 0.5 * l);
                int x_backleft = (int) (xveh - 0.5 * l);
                int x_backright = (int) (xveh - 0.5 * l);

                int y_frontleft = (int) (yveh - 0.5 * w);
                int y_frontright = (int) (yveh + 0.5 * w);
                int y_backleft = (int) (yveh - 0.5 * w);
                int y_backright = (int) (yveh + 0.5 * w);

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            } else { // access section, en la interseccion
                double xs_m = (CRUCE); // start merging
                double ys_m = ystart / scale; // start merging
                double xveh = rdlen + scale * (xs_m - 2 * LARGO_CRUCE + ca * (xs_m - pos_m - 0.5 * vehLen_m));
                double yveh = 0.5 * lw + scale * (ys_m - sa * (xs_m - pos_m - 0.5 * vehLen_m));

                int x_frontleft = (int) (xveh + 0.5 * (ca * l - sa * w));
                int x_frontright = (int) (xveh + 0.5 * (ca * l + sa * w));
                int x_backleft = (int) (xveh + 0.5 * (-ca * l - sa * w));
                int x_backright = (int) (xveh + 0.5 * (-ca * l + sa * w));

                int y_frontleft = (int) (yveh + 0.5 * (-sa * l - ca * w));
                int y_frontright = (int) (yveh + 0.5 * (-sa * l + ca * w));
                int y_backleft = (int) (yveh + 0.5 * (sa * l - ca * w));
                int y_backright = (int) (yveh + 0.5 * (sa * l + ca * w));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }

        } // end constructor
    } // end PolyVehToMerida

    class PolyVehToValle {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public PolyVehToValle(double pos_m,
                double vehLen_m, double vehWidth_m, int rdlen) {

            double l = vehLen_m * scale; // length of drawn car
            double w = vehWidth_m * scale; // width of drawn car
            if (pos_m > (CRUCE)) { // actual merging section
                double sa = Math.sin(ANGLE_ACCESSFV);
                double ca = Math.cos(ANGLE_ACCESSFV);
                double xs_m = (CRUCE); // start merging
                double ys_m = ystart / scale; // start merging
                double xveh = (scale) * STRAIGHT_FROMMERIDA_M + xstart + scale * (xs_m - ca * (xs_m - pos_m - 0.5 * vehLen_m));
                double yveh = 0.5 * lw + scale * (ys_m + sa * (xs_m - pos_m - 0.5 * vehLen_m));


                int x_frontleft = (int) (xveh + 0.5 * (ca * l - sa * w));
                int x_frontright = (int) (xveh + 0.5 * (ca * l + sa * w));
                int x_backleft = (int) (xveh + 0.5 * (-ca * l - sa * w));
                int x_backright = (int) (xveh + 0.5 * (-ca * l + sa * w));

                int y_frontleft = (int) (yveh + 0.5 * (-sa * l - ca * w));
                int y_frontright = (int) (yveh + 0.5 * (-sa * l + ca * w));
                int y_backleft = (int) (yveh + 0.5 * (sa * l - ca * w));
                int y_backright = (int) (yveh + 0.5 * (sa * l + ca * w));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            } else { // access section, pos < CRUCE
                double sa = Math.sin(ANGLE_ACCESS);
                double ca = Math.cos(ANGLE_ACCESS);
                double xs_m = (0.); // start merging
                double ys_m = ystart / scale; // start merging
                double xveh = (scale) * STRAIGHT_FROMMERIDA_M + xstart + scale * (xs_m - ca * (xs_m - pos_m - 0.25 * vehLen_m));
                double yveh = 1.5 * lw + scale * (ys_m + sa * (xs_m - pos_m - 0.25 * vehLen_m));


                int x_frontleft = (int) (xveh + 0.5 * (ca * l - sa * w));
                int x_frontright = (int) (xveh + 0.5 * (ca * l + sa * w));
                int x_backleft = (int) (xveh + 0.5 * (-ca * l - sa * w));
                int x_backright = (int) (xveh + 0.5 * (-ca * l + sa * w));

                int y_frontleft = (int) (yveh + 0.5 * (-sa * l - ca * w));
                int y_frontright = (int) (yveh + 0.5 * (-sa * l + ca * w));
                int y_backleft = (int) (yveh + 0.5 * (sa * l - ca * w));
                int y_backright = (int) (yveh + 0.5 * (sa * l + ca * w));

                xPoints[0] = x_frontright;
                xPoints[1] = x_frontleft;
                xPoints[2] = x_backleft;
                xPoints[3] = x_backright;
                xPoints[4] = x_frontright;

                yPoints[0] = y_frontright;
                yPoints[1] = y_frontleft;
                yPoints[2] = y_backleft;
                yPoints[3] = y_backright;
                yPoints[4] = y_frontright;
            }

        } // end constructor
    } // fin To Valle

    class PolyVehFromValle {

        public int xPoints[] = new int[5];
        public int yPoints[] = new int[5];

        public PolyVehFromValle(double pos_m,
                double vehLen_m, double vehWidth_m, int rdlen) {

            double l = vehLen_m * scale; // length of drawn car
            double w = vehWidth_m * scale; // width of drawn car
            //double cruce = (Math.sqrt(Math.pow(ANCHO_CRUCE,2)+Math.pow(LARGO_CRUCE,2)));
            int rdlen_m = (int) (rdlen / scale);//EN METROS
            double d2;
            if (choice_Szen == 7 || choice_Szen == 9) {
                d2 = rdlen_m - LARGO_CRUCE;
            } else {
                d2 = rdlen_m - STRAIGHT_FROMMERIDA_M - LARGO_CRUCE;
            }


            //if (pos_m >= 0 && pos_m < d1) { //bajando: primera recta inclinada
            if (pos_m >= 0 && pos_m < d2) {
                if (choice_Szen == 7 || choice_Szen == 9) {
                    double xs_m = rdlen_m;//rdlen_m / 2 + STRAIGHT_FROMMERIDA_M / 2 + LARGO_CRUCE / 2;//STRAIGHT_FROMMERIDA_; // start merging
                    double ys_m = ystart / scale; // start merging
                    double sa = Math.sin(ANGLE_ACCESSFV);
                    double ca = Math.cos(-ANGLE_ACCESSFV);
                    double xveh = scale * (STRAIGHT_FROMMERIDA_M + ca * (xs_m - pos_m - 0.5 * vehLen_m));
                    double yveh = -0.5 * lw + scale * (ys_m - sa * (xs_m - pos_m - 0.5 * vehLen_m));

                    int x_frontleft = (int) (xveh + 0.5 * (ca * l - sa * w));
                    int x_frontright = (int) (xveh + 0.5 * (ca * l + sa * w));
                    int x_backleft = (int) (xveh + 0.5 * (-ca * l - sa * w));
                    int x_backright = (int) (xveh + 0.5 * (-ca * l + sa * w));

                    int y_frontleft = (int) (yveh + 0.5 * (-sa * l - ca * w));
                    int y_frontright = (int) (yveh + 0.5 * (-sa * l + ca * w));
                    int y_backleft = (int) (yveh + 0.5 * (sa * l - ca * w));
                    int y_backright = (int) (yveh + 0.5 * (sa * l + ca * w));

                    xPoints[0] = x_frontright;
                    xPoints[1] = x_frontleft;
                    xPoints[2] = x_backleft;
                    xPoints[3] = x_backright;
                    xPoints[4] = x_frontright;

                    yPoints[0] = y_frontright;
                    yPoints[1] = y_frontleft;
                    yPoints[2] = y_backleft;
                    yPoints[3] = y_backright;
                    yPoints[4] = y_frontright;
                }

                if (choice_Szen == 8 || choice_Szen == 10) {
                    double xs_m = rdlen_m;//rdlen_m / 2 + STRAIGHT_FROMMERIDA_M / 2 + LARGO_CRUCE / 2;//STRAIGHT_FROMMERIDA_; // start merging
                    double ys_m = ystart / scale; // start merging
                    double sa = Math.sin(ANGLE_ACCESSFV);
                    double ca = Math.cos(-ANGLE_ACCESSFV);
                    double xveh = scale * (ca * (xs_m - pos_m - 0.5 * vehLen_m));
                    double yveh = -0.25 * lw + scale * (ys_m - sa * (xs_m - pos_m - 0.25 * vehLen_m));

                    int x_frontleft = (int) (xveh + 0.5 * (ca * l - sa * w));
                    int x_frontright = (int) (xveh + 0.5 * (ca * l + sa * w));
                    int x_backleft = (int) (xveh + 0.5 * (-ca * l - sa * w));
                    int x_backright = (int) (xveh + 0.5 * (-ca * l + sa * w));

                    int y_frontleft = (int) (yveh + 0.5 * (-sa * l - ca * w));
                    int y_frontright = (int) (yveh + 0.5 * (-sa * l + ca * w));
                    int y_backleft = (int) (yveh + 0.5 * (sa * l - ca * w));
                    int y_backright = (int) (yveh + 0.5 * (sa * l + ca * w));

                    xPoints[0] = x_frontright;
                    xPoints[1] = x_frontleft;
                    xPoints[2] = x_backleft;
                    xPoints[3] = x_backright;
                    xPoints[4] = x_frontright;

                    yPoints[0] = y_frontright;
                    yPoints[1] = y_frontleft;
                    yPoints[2] = y_backleft;
                    yPoints[3] = y_backright;
                    yPoints[4] = y_frontright;
                }
            } else if (pos_m >= d2) {// bajando: en la interseccion
                if (choice_Szen == 7 || choice_Szen == 9) {
                    double xs_m = d2; // start merging
                    double ys_m = ystart / scale; // start merging
                    double sa = Math.sin(ANGLE_ACCESSFVI);
                    double ca = Math.cos(-ANGLE_ACCESSFVI);
                    double xveh = scale * (STRAIGHT_FROMMERIDA_M + LARGO_CRUCE + ca * (xs_m - pos_m - 0.5 * vehLen_m));
                    double yveh = -0.5 * lw + scale * (ys_m - sa * (xs_m - pos_m - 0.5 * vehLen_m));

                    int x_frontleft = (int) (xveh + 0.5 * (ca * l - sa * w));
                    int x_frontright = (int) (xveh + 0.5 * (ca * l + sa * w));
                    int x_backleft = (int) (xveh + 0.5 * (-ca * l - sa * w));
                    int x_backright = (int) (xveh + 0.5 * (-ca * l + sa * w));

                    int y_frontleft = (int) (yveh + 0.5 * (-sa * l - ca * w));
                    int y_frontright = (int) (yveh + 0.5 * (-sa * l + ca * w));
                    int y_backleft = (int) (yveh + 0.5 * (sa * l - ca * w));
                    int y_backright = (int) (yveh + 0.5 * (sa * l + ca * w));

                    xPoints[0] = x_frontright;
                    xPoints[1] = x_frontleft;
                    xPoints[2] = x_backleft;
                    xPoints[3] = x_backright;
                    xPoints[4] = x_frontright;

                    yPoints[0] = y_frontright;
                    yPoints[1] = y_frontleft;
                    yPoints[2] = y_backleft;
                    yPoints[3] = y_backright;
                    yPoints[4] = y_frontright;
                }
                if (choice_Szen == 8 || choice_Szen == 10) { // recta desde la interseccion
                    int xveh = rdlen - (int) (scale * (pos_m + 0.5 * vehLen_m));
                    double yveh = ystart - lw * 0.5;


                    int x_frontleft = (int) (xveh + 0.5 * l);
                    int x_frontright = (int) (xveh + 0.5 * l);
                    int x_backleft = (int) (xveh - 0.5 * l);
                    int x_backright = (int) (xveh - 0.5 * l);

                    int y_frontleft = (int) (yveh - 0.5 * w);
                    int y_frontright = (int) (yveh + 0.5 * w);
                    int y_backleft = (int) (yveh - 0.5 * w);
                    int y_backright = (int) (yveh + 0.5 * w);

                    xPoints[0] = x_frontright;
                    xPoints[1] = x_frontleft;
                    xPoints[2] = x_backleft;
                    xPoints[3] = x_backright;
                    xPoints[4] = x_frontright;

                    yPoints[0] = y_frontright;
                    yPoints[1] = y_frontleft;
                    yPoints[2] = y_backleft;
                    yPoints[3] = y_backright;
                    yPoints[4] = y_frontright;
                    ////////////////////////

                }

            }
        } // end constructor
    } // end PolyVehFromValle

    class DetPosU { // positioning of road signs etc as function of position

        public int x;
        public int y;

        public DetPosU(double pos) {
            int intpos = (int) (pos * scale);

            if ((intpos > l_straight) && (intpos < (rdlen - l_straight))) {

                double arc = -0.5 * (Math.PI)
                        - (2 * (Math.PI) * ((double) (intpos - l_straight)))
                        / ((double) (2 * (rdlen - 2 * l_straight)));

                double ca = Math.cos(arc);
                double sa = Math.sin(arc);


                //x = (int) (xc + radius * ca);
                //y = (int) (yc + radius * sa);
            }
            if (intpos <= l_straight) {
                //x = (int) xc + l_straight - intpos;
                //y = (int) yc - radius;
            }
            if (intpos > (rdlen - l_straight)) {
                //x = (int) (xc + l_straight + intpos - rdlen);
                //y = (int) (yc + radius);
            }
        }
    }

    class DetPosS { // positioning of road signs etc as function of position
// Straight roads

        public int x;
        public int y;

        public DetPosS(double pos_m, int yc) {
            x = (int) (int) (pos_m * scale);
            y = yc;
        }
    } // end DetPosS

    class DetPosC {

        public int x;
        public int y;

        public DetPosC(double pos_m) {

            double arc = -pos_m / RADIUS_M - 0.25 * (Math.PI);
            double ca = Math.cos(arc);
            double sa = Math.sin(arc);

//            x = (int) (xc + radius * ca); // radius in pixels
            //          y = (int) (yc + radius * sa);
        }

        public DetPosC(double pos_m, double rightOffset_m) {
            double rightOffset = scale * rightOffset_m;
            double arc = -pos_m / RADIUS_M - 0.25 * (Math.PI);
            double ca = Math.cos(arc);
            double sa = Math.sin(arc);

//            x = (int) (xc + (radius + rightOffset) * ca);
            // y = (int) (yc + (radius + rightOffset) * sa);
        }
    }

    private void setInsetDiagrams() {
        final double CIRCUMF_M = 2. * Math.PI * RADIUS_M;

        if (choice_Szen == 1) {
            det1_0 = new TimeAvDet(0.125 * CIRCUMF_M, 30.0, 0);
            det1_1 = new TimeAvDet(0.125 * CIRCUMF_M, 30.0, 1);
            det2_0 = new SpaceAvDet(0.125 * CIRCUMF_M, 50.0, 0);
            det2_1 = new SpaceAvDet(0.125 * CIRCUMF_M, 50.0, 1);
            det3_0 = new TimeAvDet(0.375 * CIRCUMF_M, 30.0, 0);
            det3_1 = new TimeAvDet(0.375 * CIRCUMF_M, 30.0, 1);
            det4_0 = new SpaceAvDet(0.375 * CIRCUMF_M, 50.0, 0);
            det4_1 = new SpaceAvDet(0.375 * CIRCUMF_M, 50.0, 1);
            det5_0 = new TimeAvDet(0.625 * CIRCUMF_M, 30.0, 0);
            det5_1 = new TimeAvDet(0.625 * CIRCUMF_M, 30.0, 1);
            det6_0 = new SpaceAvDet(0.625 * CIRCUMF_M, 50.0, 0);
            det6_1 = new SpaceAvDet(0.625 * CIRCUMF_M, 50.0, 1);
            det7_0 = new TimeAvDet(0.875 * CIRCUMF_M, 30.0, 0);
            det7_1 = new TimeAvDet(0.875 * CIRCUMF_M, 30.0, 1);
            det8_0 = new SpaceAvDet(0.875 * CIRCUMF_M, 50.0, 0);
            det8_1 = new SpaceAvDet(0.875 * CIRCUMF_M, 50.0, 1);

            diagram1 = createImage(diaWidth, diaHeight);
            diagram2 = createImage(diaWidth, diaHeight);
            diagram3 = createImage(diaWidth, diaHeight);
            diagram4 = createImage(diaWidth, diaHeight);

            fcdxdv = new StatBufDiag(-10.0, 10.0, 0.0, 100.0,
                    diaWidth, diaHeight, 400,
                    Color.white, Color.black, diagram1,
                    "Velocity Difference", "Headway", "",
                    "Blue Car", 5.0, "5 m/s", 50.0, "50 m");
            String[] ylabels = {"acc", "V"};
            Color[] colors = {Color.black, Color.red};
            double[] ytics = {4.0, 2.86};
            String[] yticLabels = {"4 m/s^2", "20 m/s"};
            fcvt = new DynBufDiag(0.0, 60.0, -5.0, 5.0, diaWidth, diaHeight,
                    60.0, Color.white, Color.black, diagram2, "time", ylabels,
                    colors, "", "Acceleration+Velocity",
                    30.0, "-30s", ytics, yticLabels);
            Qrho = new StatBufDiag(0.0, 0.15, 0.0, 0.8, diaWidth, diaHeight,
                    2000, Color.white, Color.black, diagram3,
                    "rho", "Q",
                    "Detectors D1-D4", "Data from",
                    0.075, "75 Veh/km", 0.417, "1500 Veh/h");
            QrhoS = new StatBufDiag(0.0, 0.15, 0.0, 0.8, diaWidth, diaHeight,
                    1000, Color.white, Color.black, diagram4,
                    "rho", "Q",
                    "Detectors D1-D4", "Actual Data",
                    0.075, "75 Veh/km", 0.417, "1500 Veh/h");

        }
        if (choice_Szen == 5) {

            det1_0 = new TimeAvDet(0.8 * CIRCUMF_M, 30.0, 0);
            det1_1 = new TimeAvDet(0.8 * CIRCUMF_M, 30.0, 1);

            diagram1 = createImage(diaWidth, diaHeight);
            diagram2 = createImage(diaWidth, diaHeight);

            String[] ylabels = {"acc", "V"};
            Color[] colors = {Color.black, Color.red};
            double[] ytics = {4.0, 2.86};
            String[] yticLabels = {"4 m/s^2", "20 m/s"};
            fcvt = new DynBufDiag(0.0, 120.0, -5.0, 5.0, diaWidth, diaHeight,
                    120.0, Color.white, Color.black, diagram1,
                    "time",
                    ylabels, colors, "Blue Car",
                    "Acceleration+Velocity",
                    60.0, "-60s", ytics, yticLabels);
            String[] ylabels1 = {"Q"};
            Color[] colors1 = {Color.black};
            double[] ytics1 = {0.417};
            String[] yticLabels1 = {"1500 Veh/h"};

            Qt = new DynBufDiag(0.0, 60.0, 0.0, 0.8, diaWidth, diaHeight,
                    60.0, Color.white, Color.black, diagram2, "time",
                    ylabels1, colors1, "", "Total Flow at D1",
                    30.0, "-30s", ytics1, yticLabels1);
        }

        if ((choice_Szen == 2) || (choice_Szen == 3) || (choice_Szen == 4)) {

            det1_0 = new TimeAvDet(0.9 * CIRCUMF_M, 30.0, 0);
            det1_1 = new TimeAvDet(0.9 * CIRCUMF_M, 30.0, 1);
            det2_0 = new SpaceAvDet(0.9 * CIRCUMF_M, 50.0, 0);
            det2_1 = new SpaceAvDet(0.9 * CIRCUMF_M, 50.0, 1);
            if (choice_Szen == 3) {
                det3_0 = new TimeAvDet(0.6 * CIRCUMF_M, 30.0, 0);
                det3_1 = new TimeAvDet(0.6 * CIRCUMF_M, 30.0, 1);
                det4_0 = new SpaceAvDet(0.6 * CIRCUMF_M, 50.0, 0);
                det4_1 = new SpaceAvDet(0.6 * CIRCUMF_M, 50.0, 1);
            } else {
                det3_0 = new TimeAvDet(0.4 * CIRCUMF_M, 30.0, 0);
                det3_1 = new TimeAvDet(0.4 * CIRCUMF_M, 30.0, 1);
                det4_0 = new SpaceAvDet(0.4 * CIRCUMF_M, 50.0, 0);
                det4_1 = new SpaceAvDet(0.4 * CIRCUMF_M, 50.0, 1);
            }
            diagram1 = createImage(diaWidth, diaHeight);
            diagram2 = createImage(diaWidth, diaHeight);
            diagram3 = createImage(diaWidth, diaHeight);
            diagram4 = createImage(diaWidth, diaHeight);

            // martin nov07: funddia echt gezeigt? sonst Qt(t)
            // aktiviere mit g.drawImage
            Qrho = new StatBufDiag(0.0, 0.15, 0.0, 0.8, diaWidth, diaHeight, 500,
                    Color.white, Color.black, diagram1, "rho", "Q",
                    "Detector at D2", "Measured Data",
                    0.075, "75 Veh/km", 0.417, "1500 Veh/h");


            if (false) {
                // martin nov07: aktiviere, falls Q(t) getzeigt
                // aktiviere mit g.drawImage

                String[] ylabels1 = {"Q"};
                Color[] colors1 = {Color.black};
                double[] ytics1 = {0.417};
                String[] yticLabels1 = {"1500 Veh/h"};


                Qt = new DynBufDiag(0.0, 0.15, 0.0, 0.8, diaWidth, diaHeight,
                        60.0, Color.white, Color.black, diagram1, "time",
                        ylabels1, colors1, "", "Total Flow at D1",
                        30.0, "-30s", ytics1, yticLabels1);
            }


            QrhoS = new StatBufDiag(0.0, 0.15, 0.0, 0.8, diaWidth, diaHeight, 100,
                    Color.white, Color.black, diagram2, "rho", "Q",
                    "Detector at D2", "Actual Data",
                    0.075, "75 Veh/km", 0.417, "1500 Veh/h");
            Qrho1 = new StatBufDiag(0.0, 0.15, 0.0, 0.8, diaWidth, diaHeight, 500,
                    Color.white, Color.black, diagram3, "rho", "Q",
                    "Detector at D1", "Measured Data",
                    0.075, "75 Veh/km", 0.417, "1500 Veh/h");
            QrhoS1 = new StatBufDiag(0.0, 0.15, 0.0, 0.8, diaWidth, diaHeight, 100,
                    Color.white, Color.black, diagram4, "rho", "Q",
                    "Detector at D1", "Actual Data",
                    0.075, "75 Veh/km", 0.417, "1500 Veh/h");

        }
    }

    private void updateInsetDiagrams() {
        if (choice_Szen == 1) {
            det1_0.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det1_1.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det2_0.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det2_1.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det3_0.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det3_1.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det4_0.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det4_1.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det5_0.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det5_1.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det6_0.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det6_1.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det7_0.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det7_1.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det8_0.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det8_1.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);


            double fcdx = (microstreet.fcd) - 5.0;
            double fcvd = -(microstreet.fcvd);
            double fcacc = microstreet.fcacc;
            double fcv = microstreet.fcvel;
            fcvt.addPoint(time, fcacc, 0);
            fcvt.addPoint(time, (fcv / 7.0), 1);
            fcdxdv.addPoint(fcvd, fcdx);

            double Q = det1_0.flow();
            double V = det1_0.harmVel();
            double rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }
            Q = det1_1.flow();
            V = det1_1.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }
            Q = det3_0.flow();
            V = det3_0.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }
            Q = det3_1.flow();
            V = det3_1.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }
            Q = det5_0.flow();
            V = det5_0.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }
            Q = det5_1.flow();
            V = det5_1.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }
            Q = det7_0.flow();
            V = det7_0.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }
            Q = det7_1.flow();
            V = det7_1.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }

            rho = det2_0.density();
            V = det2_0.avVel();
            QrhoS.addPoint(rho, rho * V);
            rho = det2_1.density();
            V = det2_1.avVel();
            QrhoS.addPoint(rho, rho * V);

            rho = det4_0.density();
            V = det4_0.avVel();
            QrhoS.addPoint(rho, rho * V);
            rho = det4_1.density();
            V = det4_1.avVel();
            QrhoS.addPoint(rho, rho * V);

            rho = det6_0.density();
            V = det6_0.avVel();
            QrhoS.addPoint(rho, rho * V);
            rho = det6_1.density();
            V = det6_1.avVel();
            QrhoS.addPoint(rho, rho * V);

            rho = det8_0.density();
            V = det8_0.avVel();
            QrhoS.addPoint(rho, rho * V);
            rho = det8_1.density();
            V = det8_1.avVel();
            QrhoS.addPoint(rho, rho * V);
        }

        if (choice_Szen == 5) {

            det1_0.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det1_1.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);

            double fcdx = (microstreet.fcd) - 5.0;
            double fcvd = -(microstreet.fcvd);
            double fcacc = microstreet.fcacc;
            double fcv = microstreet.fcvel;

            double Q = (det1_0.flow());
            Q = Q + (det1_1.flow());

            fcvt.addPoint(time, fcacc, 0);
            fcvt.addPoint(time, (fcv / 7.0), 1);
            //fcdxdv.addPoint(fcvd, fcdx);
            Qt.addPoint(time, Q, 0);
        }

        if ((choice_Szen == 2) || (choice_Szen == 3) || (choice_Szen == 4)) {

            det1_0.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det1_1.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det2_0.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det2_1.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);

            det3_0.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det3_1.update(microstreet.positions, microstreet.old_pos,
                    microstreet.velocities, microstreet.lanes, time);
            det4_0.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);
            det4_1.update(microstreet.positions, microstreet.distances,
                    microstreet.velocities, microstreet.lanes);

            double Q = det1_0.flow();
            double V = det1_0.harmVel();
            double rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }
            Q = det1_1.flow();
            V = det1_1.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho.addPoint(rho, Q);
            }

            Q = det3_0.flow();
            V = det3_0.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho1.addPoint(rho, Q);
            }
            Q = det3_1.flow();
            V = det3_1.harmVel();
            rho = 0;
            if (V > 0.0) {
                rho = Q / V;
                Qrho1.addPoint(rho, Q);
            }

            rho = det2_0.density();
            V = det2_0.avVel();
            QrhoS.addPoint(rho, rho * V);
            rho = det2_1.density();
            V = det2_1.avVel();
            QrhoS.addPoint(rho, rho * V);

            rho = det4_0.density();
            V = det4_0.avVel();
            QrhoS1.addPoint(rho, rho * V);
            rho = det4_1.density();
            V = det4_1.avVel();
            QrhoS1.addPoint(rho, rho * V);

        }
    }

    private void paintInsetDiagrams(Graphics g) {
        final double CIRCUMF_M = 2. * Math.PI * RADIUS_M;

        if (choice_Szen == 1) {

            // actual diagrams

            g.drawImage(fcdxdv.picture(), dia1x, dia2y, null);
            g.drawImage(fcvt.picture(), dia2x, dia2y, null);
            g.drawImage(Qrho.picture(), dia3x, dia3y, null);
            g.drawImage(QrhoS.picture(), dia4x, dia4y, null);

            // labels "D1" (top) ... "D4" (right)

            int detx, dety;
            int distance = (int) (15.0 * scale); // distance from road (15 m)
            int xoff = metricsText.stringWidth("D2");
            int yoff = (int) (0.8 * metricsText.stringWidth("M"));
            g.setFont(textFont);
            g.setColor(Color.black);

            detx = (new DetPosC(0.125 * CIRCUMF_M)).x;
            dety = (new DetPosC(0.125 * CIRCUMF_M)).y;
            g.drawString("D1", detx - (int) (0.5 * xoff), dety + distance + yoff);

            detx = (new DetPosC(0.375 * CIRCUMF_M)).x;
            dety = (new DetPosC(0.375 * CIRCUMF_M)).y;
            g.drawString("D2", detx + distance, dety + (int) (0.5 * yoff));

            detx = (new DetPosC(0.625 * CIRCUMF_M)).x;
            dety = (new DetPosC(0.625 * CIRCUMF_M)).y;
            g.drawString("D3", detx - (int) (0.5 * xoff), dety - distance);

            detx = (new DetPosC(0.875 * CIRCUMF_M)).x;
            dety = (new DetPosC(0.875 * CIRCUMF_M)).y;
            g.drawString("D4", detx - distance - xoff, dety + (int) (0.5 * yoff));

        }

        // traffic light scenario; 2 diagrams

        if (choice_Szen == 5) {
            g.drawImage(fcvt.picture(), dia1x, dia1y, null);
            g.drawImage(Qt.picture(), dia2x, dia2y, null);


            // label "D1" at the position of the detector

            double det1pos = 0.8 * CIRCUMF_M;
            int detx = (new DetPosU(det1pos)).x;
            int dety = (new DetPosU(det1pos)).y;
            g.setFont(textFont);
            g.setColor(Color.black);
            g.drawString("D1", detx, dety);


        }


        if ((choice_Szen == 2) || (choice_Szen == 3) || (choice_Szen == 4)) {

            g.drawImage(Qrho1.picture(), dia1x, dia1y, null);
            g.drawImage(QrhoS1.picture(), dia2x, dia2y, null);
            g.drawImage(Qrho.picture(), dia3x, dia3y, null);
            g.drawImage(Qrho.picture(), dia4x, dia4y, null);
            //g.drawImage (Qt.picture(), dia4x,dia4y,null); //martin nov07

            double detpos = (choice_Szen == 4) ? 0.4 * CIRCUMF_M : 0.6 * CIRCUMF_M;
            int detx = (new DetPosU(detpos)).x;
            int dety = (new DetPosU(detpos)).y;
            g.setFont(textFont);
            g.setColor(Color.black);
            g.drawString("D1", detx, dety);
            detpos = 0.9 * CIRCUMF_M;
            detx = (new DetPosU(detpos)).x;
            dety = (new DetPosU(detpos)).y;
            g.setFont(textFont);
            g.setColor(Color.black);
            g.drawString("D2", detx, dety);
        }

    }
    // end paintInsetDiagrams
}
