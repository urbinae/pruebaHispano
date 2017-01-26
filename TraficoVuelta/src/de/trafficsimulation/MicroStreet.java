package de.trafficsimulation;

import java.awt.Color;
import java.util.Random;
import java.util.Vector;

/**
Representation of a two-lane road section for one direction.
The main elements of MicroStreet are 
<ul>
<li> street, a vector of Moveable's representiung the vehicles,
<li> The update method invoked in every time step. Ammong others, it calls all emthods mentioned below.
<li> Methods for moving the vehicles (translate), accelerating them (accelerate) and performing the lane changes (changeLanes).
<li> A sorting routine sort for rearranging the vehicle order in street in the order of decreasing longitudinal positions
<li> The method ioFlow implementing the upstrea and downstream boundary conditions (inflow and outflow).
</ul>
<br><br>
The realization of an on ramp, the class Onramp, is derived from this class.
 */
public class MicroStreet implements Constants {

    static final int LEFT = 0;
    static final int RIGHT = 1;
    //######## colors <martin nov07>
    static final Color colorCar=new Color(0,130,230); //Color(r,g,b):rojo, verde y azul
    static final Color colorCarFM = new Color(40, 40, 60);
    static final Color colorCarTT = new Color(40, 40, 60);
    static final Color colorCarTV = new Color(255, 130, 0);
    
    static final Color colorTruck = new Color(40, 40, 60);
    static final Color colorPerturb = new Color(255, 130, 0);
    // vector of Moveables
    //final int IMAXINIT = (int)(0.001*RADIUS_M * 4 * Math.PI * DENS_MAX_INVKM + 10); // = 130,637157898
    final int IMAXINIT = 100;
    protected Vector street = new Vector(IMAXINIT);
    //Para llevar las estadisticas de los tiempos de llegada y de servicio
    
        Vector arrive = new Vector(IMAXINIT);
        Vector exit = new Vector(IMAXINIT);
    
    // vector of data of Moveables for output (only model is missing)
    public Vector positions = new Vector(IMAXINIT);
    public Vector velocities = new Vector(IMAXINIT);
    public Vector numbers = new Vector(IMAXINIT);
    public Vector lanes = new Vector(IMAXINIT);
    public Vector colors = new Vector(IMAXINIT);
    public Vector lengths = new Vector(IMAXINIT);
    public Vector timeServ = new Vector(IMAXINIT);
    // additional vectors for output
    public Vector distances = new Vector(IMAXINIT);
    public Vector old_pos = new Vector(IMAXINIT);
    public Vector old_lanes = new Vector(IMAXINIT);
    public Vector old_numbers = new Vector(IMAXINIT);
    // info if cars are removed in closed system
    // informacion si los carros se retiran del sistema cerrado
    public boolean circleCarsRemoved = false;
    // floating car data
    public double fcd = 0.0;            // distance
    public double fcvd = 0.0;           // approaching rate
    public double fcvel = 0.0;          // v
    public double fcacc = 0.0;          // acceleration
    public int fcnr = 0;
    public boolean red = false;
    // longitudinal models; 
    // sync* types only needed for choice_Szen==4; defined there 
    protected MicroModel idmCar = new IDMCar();
    protected MicroModel idmTruck = new IDMTruck();
    protected MicroModel sync1Car;
    protected MicroModel sync2Car;
    protected MicroModel sync1Truck;
    protected MicroModel sync2Truck;
        
    //Modelo IDM al inicio del canal From Tabay
    protected MicroModel idmCarFT = new IDMCarFT(); 
    protected MicroModel idmTruckFT = new IDMTruckFT();
    
    // para las curvas. Subiendo hacia Merida
    protected MicroModel syncCarFT = new IDMSyncCarFT();
    protected MicroModel syncTruckFT = new IDMSyncTruckFT();
               
    //Modelo IDM al inicio del canal From Merida
    protected MicroModel idmCarFM = new IDMCarFM(); 
    protected MicroModel idmTruckFM = new IDMTruckFM();
    
    // Modelo hacia el valle, distinto del modelo desde merida
    protected MicroModel idmCarTV = new IDMCarTV(); 
    protected MicroModel idmTruckTV = new IDMTruckTV();
    
    // Modelo hacia Tabay, distinto del modelo desde merida
    protected MicroModel idmCarTT = new IDMCarTT(); 
    protected MicroModel idmTruckTT = new IDMTruckTT();
    
    // para las curvas. Bajando hacia Tabay
    protected MicroModel syncCarTT = new IDMSyncCarTT();
    protected MicroModel syncTruckTT = new IDMSyncTruckTT();
    
    //Modelo IDM al inicio del canal From Valle
    protected MicroModel idmCarFV = new IDMCarFV(); 
    protected MicroModel idmTruckFV = new IDMTruckFV();
    
    //Modelo IDM al final de canal hacia Merida
    protected MicroModel syncCarTM = new IDMSyncCarTT();
    protected MicroModel syncTruckTM = new IDMSyncTruckTM();
    
    
    // lane-change models  (p, db, smin, bsave)
    //!!! truck (=> vw for better impl!)
    protected LaneChange polite = new LaneChange(P_FACTOR_TRUCK,
            DB_TRUCK, MAIN_SMIN, MAIN_BSAVE, BIAS_RIGHT_TRUCK);
    //!!! car (=> vw for better impl!)
    // P_FACTOR_CAR=0.2, factor de cortesia
    // DB_CAR=0.3, changing thresholds (m/s^2), umbral de cambio de desaceleracion
    // MAIN_SMIN=2, brecha minima en la ruta principal
    // MAIN_BSAVE=12, limite seguridad principal
    // BIAS_RIGHT_CAR=0.1, sesgo para carros entre los carriles
    protected LaneChange inconsiderate = new LaneChange(P_FACTOR_CAR,
            DB_CAR, MAIN_SMIN, MAIN_BSAVE, BIAS_RIGHT_CAR);
    // neighbours
    protected Moveable vL, vR, hL, hR;
    // can be modified interactively
    // mt apr05: seed 42 eingefuehrt wie in 3dsim 
    protected Random random = new Random(42); // for truck perc. and veh. numbers
    protected int choice_Szen;
    protected int choice_Geom; // circle or U
    protected double roadLength; // length depends on choice_Szen 
    protected double upcurve1; // location where flow-conserving bottleneck begins
    
    protected double upcurve2;
    protected double interFV;
    protected double cercaInter;
    // time and simulation control
    //protected double intersection;
    protected double time;
    
    //Estadisticas para los tiempos de servicio
    public int numVehFMTT=0;
    public int numVehFMTV=0;
    public int numVehFT=0;
    public int numVehFV=0;
    public double sumTimesFMTV=0.;
    public double sumTimesFMTT=0.;
    public double sumTimesFT=0.;
    public double sumTimesFV=0.;
    public double serviceTFMTV=0;
    public double serviceTFMTT=0;
    public double serviceTFT=0;
    public double serviceTFV=0;
    // Source: nin = integral of inflow mod 1
    protected double nin = 0.0;
    
    protected int contPrior=0;

    // length: tamaño de la ruta
    // density: densidad de autos
    // p_factor: factor de cortesia
    // delta_B: desaceleracion
    // floatcar_nr: 
    public MicroStreet(double length, double density,
            double p_factor, double deltaB,
            int floatcar_nr, int choice_Szen) {
        time = 0;
        roadLength = length;
        //upcurve1 = STRAIGHT1_M - RADIUS_M;
        //intersection =roadLength;
        this.choice_Szen = choice_Szen;
        this.choice_Geom = ((choice_Szen == 1) || (choice_Szen == 6)) ? 0 : 1;

        inconsiderate.set_p(p_factor);
        inconsiderate.set_db(deltaB);
        double mult = ((choice_Geom == 0) && CLOCKWISE) ? (-1) : 1;
        double bias_right_truck = (choice_Szen == 3) ? BIAS_RIGHT_TRUCK3 : BIAS_RIGHT_TRUCK;
        double bias_right_car = (choice_Szen == 3) ? BIAS_RIGHT_CAR3 : BIAS_RIGHT_CAR;
        polite.set_biasRight(mult * bias_right_truck);
        inconsiderate.set_biasRight(mult * bias_right_car);

        fcnr = floatcar_nr;

        System.out.println("MicroStreet(args) cstr: roadLength="
                + roadLength);        
       
    }

    // ################# end constructor ##########################
// <martin nov07>
    public void applyLocalPerturbation() {
        int imax = street.size();
        int i = 5 * imax / 6;
        Moveable veh = ((Moveable) (street.elementAt(i)));
        double vel = 0.1 * veh.velocity();
        veh.setVelocity(vel);
        veh.setColor(colorPerturb);
        System.out.println("MicroStreet.applyLocalPert.: new velocity=" + vel);
    }

    public double length() {
        return roadLength;
    }

    public void setLength(double roadLength) {
        this.roadLength = roadLength;
    }
    
    // make actual state available in form of vectors over all vehicles;
    // protected methods set public vectors to be used for graphical output
    protected Vector setPos() { //convierte cada elemento del vector street en un movil y asigna su posicion en el vector
        Vector temp = new Vector(IMAXINIT); // crea un vector temporal
        //System.out.println("temp.size() = "+ temp.size());
        int imax = street.size(); //obtiene el tamaño del vector
        //System.out.println("street.size() = "+ imax);
        for (int i = 0; i < imax; i++) {
            double pos = ((Moveable) (street.elementAt(i))).position(); // cada elemento del vector es un movil y obtiene su posicion
            //System.out.println(" En setPos pos " + "["+i+"] = "+ pos);
            temp.insertElementAt(new Double(pos), i); // inserta en el vector temporal cada movil con posicion pos en el indice i de temp
        }

        return temp;// retorna el vector
    }

    // obtiene el indice del vehiculo del frente en el canal dado
    // y calcula la diferencia de la posicion entre el vehiculo del frente y el mio
    protected Vector setDistances() {  // neglect gaps in front of first/last veh
        // on either lane (i<=iFrontCarsBoth)
        // descuida las brechas en frente del primer o ultimo vehiculo en ambos canales
        Vector temp = new Vector(IMAXINIT);
        int imax = street.size();
        int iFirstLeft = firstIndexOnLane(0);
        int iFirstRight = firstIndexOnLane(1);
        int iFrontCarsBoth = (iFirstLeft > iFirstRight) // indice de los carros del frente
                ? iFirstLeft : iFirstRight;
        for (int i = 0; i <= iFrontCarsBoth + 1; i++) {  // placeholder: marcador de posicion

            temp.insertElementAt(new Double(-1.), i);
        }
        for (int i = iFrontCarsBoth + 1; i < imax; i++) {
            int lane = ((Integer) lanes.elementAt(i)).intValue();
            int iFront = nextIndexOnLane(lane, i); // obtiene el indice del vehiculo del frente en ese canal
            double distance = // diferencia de la posicion entre el vehiculo del frente y el mio
                    ((Double) positions.elementAt(iFront)).doubleValue()
                    - ((Double) positions.elementAt(i)).doubleValue();
            //System.out.println(" En setDistances " + "["+i+"] = "+distance);
            temp.insertElementAt(new Double(distance), i);
        }
        return temp;
    }

    protected Vector setNr() { // convierte cada elemento del vector street en un movil y asigna su color en el vector
        Vector temp = new Vector(IMAXINIT);
        int imax = street.size();
        for (int i = 0; i < imax; i++) {
            int nr = ((Moveable) (street.elementAt(i))).NR();
            //System.out.println(" En setNr nr " + "["+i+"] = "+nr);
            temp.insertElementAt(new Integer(nr), i);
        }
        //System.out.println(" En setNr tamaño de temp = "+temp.size());
        return temp;
    }

    // Asigna la velocidad a cada vehiculo del vector street
    protected Vector setVel() {
        Vector temp = new Vector(IMAXINIT);
        int imax = street.size();
        for (int i = 0; i < imax; i++) {
            double vel = ((Moveable) (street.elementAt(i))).velocity();
            //System.out.println(" En setVel vel " + "["+i+"] = "+vel);
            temp.insertElementAt(new Double(vel), i);
        }
        return temp;
    }

    // Asigna 0 a cada vehiculo del canal interno (izquierdo) 
    // y 1 a cada vehiculo del canal externo (derecho) del vector street
    protected Vector setLanes() { //convierte cada elemento del vector street en un movil y asigna su canal en el vector
        Vector temp = new Vector(IMAXINIT);
        int imax = street.size();
        for (int i = 0; i < imax; i++) {
            int lane = ((Moveable) (street.elementAt(i))).lane();
            //System.out.println(" En setLane lane " + "["+i+"] = "+lane);
            temp.insertElementAt(new Integer(lane), i);
        }
        return temp;
    }

    // Asigna el color rojo a cada vehiculo
    protected Vector setColors() {
        Vector temp = new Vector(IMAXINIT);
        int imax = street.size();
        for (int i = 0; i < imax; i++) {
            Color c = ((Moveable) (street.elementAt(i))).color();
            //System.out.println(" En setColor " + "["+i+"] = "+c);
            temp.insertElementAt(c, i);
        }
        return temp;
    }

    // Asigna a cada vehiculo su tamaño
    protected Vector setLengths() {
        Vector temp = new Vector(IMAXINIT);
        int imax = street.size();
        for (int i = 0; i < imax; i++) {
            Double Len = new Double(((Moveable) (street.elementAt(i))).length());
            //System.out.println(" En setLenghts Len = " + "["+i+"] = "+Len);
            temp.insertElementAt(Len, i);
        }
        return temp;
    }

  
    public void update(double time, double dt, int choice_Szen,
            double density, double qIn, double perTr,
            double p_factor, double deltaB) {

        /*System.out.println(" qIn = " + qIn);
        System.out.println(" time =" + time);
        System.out.println(" dt =" + dt);
        System.out.println(" escen = " + choice_Szen);
        System.out.println(" density = " + density);
        System.out.println(" perTr = " + perTr);
        System.out.println(" p_factor= " + p_factor);
        System.out.println(" deltaB = " + deltaB);*/

        this.time = time;
        // used in SimCanvas.java
        // old_... son de tipo Vector
        old_pos = setPos();        // need old info for detectors and drawing
        //System.out.println("1er Tamaño del Vector old_pos = "+ old_pos.size());

        old_lanes = setLanes();
        //System.out.println(" Tamaño del Vector old_Lanes = "+old_lanes.size());

        old_numbers = setNr();
        //System.out.println(" Tamaño del Vector old_numbers = "+old_numbers.size());

        //old_destinos = setDestino();

        if (choice_Szen == 3) {    // particularly aggressive for lane closing
            inconsiderate.set_p(0.);
            polite.set_db(DB_TRUCK3); //vpara camiones
            inconsiderate.set_db(DB_CAR3); // para carros
            inconsiderate.set_bsave(21.); // limite de desaceleracion
        } else {
            // Asigna factores de cortecia o de agresividad y umbral para el Cambio de canal
            inconsiderate.set_p(p_factor); // factor de cortecia P_FACTOR_CAR=0.2
            polite.set_db(DB_TRUCK); // umbral de cambio=0.2
            inconsiderate.set_db(DB_CAR); // umbral de cambio=0.3
        }
        // choice_BC=0: per. BC; otherwise open BC / BC: carros virtuales
        int choice_BC = ((choice_Szen == 1) || (choice_Szen == 6)) ? 0 : 1;

        // Main action: accelerate, changeLanes, translate, sort!
        // At least one vehicle must be on each lane / al menos un vehiculo debe estar sobre el carril

        if (true) {
            //if ((firstIndexOnLane(0)>=0)&&(firstIndexOnLane(1)>=0)){
            //insertBCCars(choice_BC);   // virtual boundary cars / carros virtules de frontera
            accelerate(dt);
            changeLanes(dt);
            clearBCCars();
        }
        translate(dt, choice_Szen);
        sort();


        positions = setPos();
        velocities = setVel();
        numbers = setNr();
        lanes = setLanes();
        colors = setColors();
        lengths = setLengths();
        distances = setDistances();

        // source terms !! export of positions etc before such that
        // old_positions and positions etc must 
        // always have same vehicle number!! -> extra var circleCarsRemoved

        ioFlow(dt, qIn, perTr, choice_BC); //needs positions etc
        if (choice_Szen == 1) {
            adaptToNewDensity(density, perTr);
        }
        //System.out.println("2do Tamaño del Vector old_pos = "+ old_pos.size());
        //System.out.println("Tamaño del Vector street = "+ street.size());
        circleCarsRemoved = ((choice_Geom == 0)
                && (street.size() < old_pos.size()));

    }

    // HIER truck => car implementieren!!
    protected void adaptToNewDensity(double density, double perTr) {
        int nCars_wished = (int) (density * roadLength * 2.0);
        int nCars = positions.size();
        if (nCars_wished > nCars) {
            System.out.println("nCars_wished=" + nCars_wished
                    + " nCars=" + nCars);
            insertOneVehicle(perTr);
        }
        if (nCars_wished < nCars) {
            System.out.println("nCars_wished=" + nCars_wished
                    + " nCars=" + nCars);
            removeOneVehicle();
        }
    }

    /*
    <Treiber aug06> Methode veraendert, so dass auch bei 0 Fz auf
    beiden Spuren neue Fz eingefuehrt werden koennen (Ringstrasse)
    </Treiber>
     */
    private void insertOneVehicle(double perTr) {

        // determine position and index of front veh

        int nveh = positions.size(); // 100
        final double mingap = 10.;
        double maxgap = 0.;
        int i_maxgap = 0;
        double pos_maxgap; // position of vehicle which maxgap in front
        int lane_maxgap; // lane of vehicle which maxgap in front

        int nleft = 0;
        int nright = 0;
        for (int i = 0; i < nveh; i++) {
            Moveable me = (Moveable) (street.elementAt(i));
            if (me.lane() == LEFT) {
                nleft++;
            } // si lane == 0, incrementa n en el canal de la izquierda
            else {
                nright++;
            } // si no, en el de la derecha
            System.out.println("i=" + i + " lane=" + me.lane() + " pos=" + (int) me.position());

            double gap = ((Double) distances.elementAt(i)).doubleValue();
            if (gap > maxgap) {
                maxgap = gap;
                i_maxgap = i;
            }
        }

        if (nleft < 2) {
            System.out.println("nleft<2!!!");
            maxgap = roadLength;
            pos_maxgap = 0;
            lane_maxgap = LEFT;
            i_maxgap = nveh;
        } else if (nright < 2) {
            System.out.println("nright<2!!!");
            maxgap = roadLength;
            pos_maxgap = 0;
            lane_maxgap = RIGHT;
            i_maxgap = nveh;
        } else {
            pos_maxgap = ((Double) positions.elementAt(i_maxgap)).doubleValue();
            lane_maxgap = ((Integer) lanes.elementAt(i_maxgap)).intValue();
        }
        System.out.println("MicroStreet.insertOneVehicle: maxgap=" + (int) maxgap
                + " index=" + i_maxgap
                + " pos=" + pos_maxgap
                + " lane=" + lane_maxgap);


        // insert vehicle if sufficient gap

        if (maxgap > mingap) {
            double rand = random.nextDouble() * 1.0;
            int randInt = Math.abs(random.nextInt());
            MicroModel modelNew = (rand < perTr) ? idmTruck : idmCar;
            LaneChange changemodelNew = (rand < perTr)
                    ? polite : inconsiderate;
            double posNew = pos_maxgap + 0.5 * maxgap;
            double vNew = modelNew.Veq(0.5 * maxgap);
            double lNew = (rand < perTr) ? LKW_LENGTH_M : PKW_LENGTH_M;
            Color colorNew = (rand < perTr)
                    ? colorTruck : colorCar;
/*            street.insertElementAt((new Car(posNew, vNew, lane_maxgap,
                    modelNew, changemodelNew,
                    lNew, colorNew, randInt)), i_maxgap);*/
        }

    }

    private void removeOneVehicle() {
        int indexToRemove = Math.abs(random.nextInt()) % (positions.size());
        street.removeElementAt(indexToRemove);
    }

    public   void insertBCCars() {
        //int i_rd = firstIndexOnLane(1);  // index right downstream vehicle
        //System.out.println("En insertBCCars : primer vehiculo en el canal 1 = " +i_rd);

        int i_ld = firstIndexOnLane(1);  //  ...
        //System.out.println("En insertBCCars : primer vehiculo en el canal 0 = " +i_ld);

        //int i_ru = lastIndexOnLane(1);  // index right upstream vehicle
        //System.out.println("En insertBCCars : ultimo vehiculo en el canal 1 = " +i_ru);
        int i_lu = lastIndexOnLane(1);
        //System.out.println("En insertBCCars : ultimo vehiculo en el canal 0 = " +i_lu);
        //System.out.println("MicroStreet.insertBCCars: i_rd="+i_rd+" i_ru="+i_ru);

        double upLeftPos = (i_lu > -1)
                ? ((Moveable) (street.elementAt(i_lu))).position() : 0;
        //System.out.println("En insertBCCars : posicion del ultimo vehiculo "+i_lu+" en el canal izquierdo = " +upLeftPos);
        //double upRightPos = (i_ru > -1)
        //      ? ((Moveable) (street.elementAt(i_ru))).position() : 0;
        //System.out.println("En insertBCCars : posicion del ultimo vehiculo "+i_ru+" en el canal derecho = " +upRightPos);
        double upLeftVel = (i_lu > -1)
                ? ((Moveable) (street.elementAt(i_lu))).velocity() : 0;
        //System.out.println("En insertBCCars : velocidad del ultimo vehiculo en el canal izquierdo = " +upLeftVel);
        //double upRightVel = (i_ru > -1)
        //      ? ((Moveable) (street.elementAt(i_ru))).velocity() : 0;
        //System.out.println("En insertBCCars : velocidad del ultimo vehiculo en el canal derecho = " +upRightVel);


        double downLeftPos = (i_ld > -1)
                ? ((Moveable) (street.elementAt(i_ld))).position() : roadLength;
        //double downRightPos = (i_rd > -1)
        //      ? ((Moveable) (street.elementAt(i_rd))).position() : roadLength;
        double downLeftVel = (i_ld > -1)
                ? ((Moveable) (street.elementAt(i_ld))).velocity() : roadLength;
        //double downRightVel = (i_rd > -1)
        //      ? ((Moveable) (street.elementAt(i_rd))).velocity() : roadLength;

        double dx = 200.;  // distance of the boundary cars
        street.insertElementAt(new BCCar(downLeftPos + dx, downLeftVel, 1, idmCar, PKW_LENGTH_M), 0);
        // street.insertElementAt(new BCCar(downRightPos + dx, downRightVel, 1, idmCar, PKW_LENGTH_M), 0);
        int imax = street.size();
        //System.out.println("En insertBCCars : tamaño del vector street en el canal 0 " +imax);
        street.insertElementAt(new BCCar(upLeftPos - dx, upLeftVel, 1, idmCar, PKW_LENGTH_M), imax);
        imax = street.size();
        //System.out.println("En insertBCCars : tamaño del vector street en el canal 1 " +imax);
        //street.insertElementAt(new BCCar(upRightPos - dx, upRightVel, 1, idmCar, PKW_LENGTH_M), imax);

    }

    protected void clearBCCars() {

        street.removeElementAt(0);
        //System.out.println("clear BCCars removeElementAt(0)= "+street.elementAt(0));
        int imax = street.size();
        imax--;
        street.removeElementAt(imax);
        //System.out.println("imax " + imax);
        //System.out.println("clear BCCars removeElementAt(0)= "+street.elementAt(imax));
    }


    protected void changeLanes(double dt) {
        final double bsave = 5.;  // maximum save braking deceleration, maxima desaceleracion de seguridad de frenado
        final double dmin = 5.;  // minimum distance necessary to change, minima distancia necesaria para cambiar
        int imax = street.size() - 3;

        for (int i = 2; i < imax; i++) {
            Moveable me = (Moveable) (street.elementAt(i));

            if (me.timeToChange(dt)) {
                int lane = me.lane();
                int newLane = ((lane == 0) ? 1 : 0);
                setNeighbours(i);                   // -> vR, hR, vL, hL
                Moveable fOld = (lane == 0) ? vL : vR; // front vehicle own lane, en mi carril
                Moveable fNew = (lane == 0) ? vR : vL; // front vehicle new lane, en el carril del lado
                Moveable bNew = (lane == 0) ? hR : hL; // back vehicle new lane, en el carril del lado

                // do actual change if incentive criterion fulfilled
                // setLane method of Moveable; setLanes M. of MicroStreet!
                // si se cumple el criterio de insentivo realizar el cambio al nuevo canal
                if (me.change(fOld, fNew, bNew)) {
                    ((Moveable) (street.elementAt(i))).setLane(newLane);
                }
            }
        }
    }
  
    protected void accelerate(double dt) {

        int imax = street.size() - 2;

        // floating car data

        fcd = 0.0;
        fcvd = 0.0;
        fcvel = 0.0;
        fcacc = 0.0;

        // Counting loop goes backwards to implement parallel update!

        for (int i = imax - 1; i >= 2; i--) {
            Moveable me = (Moveable) street.elementAt(i);
            int lane = me.lane();
            int act_nr = me.NR();
            int next_ind = nextIndexOnLane(lane, i);
            Moveable frontVeh = (Moveable) street.elementAt(next_ind);

            // if actual car = floating car, gather "detector data", reunir los datos del detector

            if (act_nr == fcnr) {
                fcd = frontVeh.position() - me.position();
                fcvd = frontVeh.velocity() - me.velocity();
                fcvel = me.velocity();
                //System.out.println(" Mi velocidad fcvel = " + fcvel);
                me.accelerate(dt, frontVeh);
                double vNew = me.velocity();
                //System.out.println("Mi nueva velocidad vNew = " + vNew);
                fcacc = (vNew - fcvel) / dt;
            } // Otherwise, do just the acceleration
            else {
                me.accelerate(dt, frontVeh);
            }
        }
    }
  

    protected int translate(double dt, int choice_Szen) {

        // without sorting

        int imax = street.size();
        for (int i = 0; i < imax; i++) {
            Moveable me = (Moveable) street.elementAt(i);
            double x_old = me.position();
            me.translate(dt);
            double x_new = me.position();

            // flow-conserving bottlenecks 
            // realized with parameter
            // gradients and uppos is crossed: 
            // Change upstream to downstream parameters

           /* if (choice_Szen == 7) {
                if ((x_old <= upcurve1) && (x_new > upcurve1)) {
                    if (me.model() == sync1Truck) {
                        me.setModel(sync2Truck);
                    } else {
                        me.setModel(sync2Car);
                    }
                }
            }*/
        }
        return street.size();
    }

    protected void ioFlow(double dt, double qIn, double perTr,
            int choice_BC) {


        // periodic BC

        if (choice_BC == 0) {
            int imax = street.size() - 1;
            if (imax >= 0) { // at least one vehicle present
                Moveable temp_car = (Moveable) street.elementAt(0);
                while (temp_car.position() > roadLength) {
                    double pos = temp_car.position();

                    // remove first vehicle
                    street.removeElementAt(0);

                    // and insert it at the end with position reduced by roadLength
                    temp_car.setPosition(pos - roadLength);
                    imax = street.size();
                    street.insertElementAt(temp_car, imax);
                    temp_car = (Moveable) street.elementAt(0);
                }
            }
        }


        // open BC

        if (choice_BC == 1) {
            int imax = street.size() - 1;
            final double spaceFree = 200.;  // v0=ve(spaceFree)
            final double spaceMin = 27; // minimum headway for new vehicle
            //System.out.println("MicroStreet.ioFlow: Anfang: imax="+imax);

            // outflow: 

            // just remove; 
            // insertBCCars always guruantees for virtual front vehicles

            if (imax >= 0) {
                // System.out.println("MicroStreet.ioFlow: removing vehicle...");
                while ((imax >= 0) && // (imax>=0) first to prevent indexOutOf...!
                        (((Moveable) (street.elementAt(0))).position() > roadLength)) {
                    street.removeElementAt(0);
                    imax--;
                }
            }

            // inflow

            //System.out.println("MicroStreet.ioFlow: idmCar.v0="+((IDMCar)(idmCar)).v0);
            //System.out.println("MicroStreet.ioFlow: beginn inflow: imax="+imax);

            nin = nin + qIn * dt;

            if (nin > 1.0) {  // new vehicle imposed by the inflow BC
                nin = nin - 1.0;
                int i_lu = lastIndexOnLane(0);  // lu = "left upper"
                int i_ru = lastIndexOnLane(1);

                // at least 1 vehicle on either lane

                if ((i_lu >= 0) && (i_ru >= 0)) {
                    imax = street.size() - 1;
                    int laneLastVeh =
                            ((Moveable) (street.elementAt(imax))).lane();
                    int lane = (laneLastVeh == 0) ? 1 : 0;  // insert on other lane
                    int iPrev = lastIndexOnLane(lane); // index of previous vehicle
                    double space = ((Moveable) (street.elementAt(iPrev))).position();
                    double vPrev = ((Moveable) (street.elementAt(iPrev))).velocity();

                    // enough space for new vehicle to enter? (!red)

                    if (!(red = (space < spaceMin))) {
                        MicroModel carmodel = (choice_Szen != 4)
                                ? idmCar : sync1Car;
                        MicroModel truckmodel = (choice_Szen != 4)
                                ? idmTruck : sync1Truck;
                        double rand = random.nextDouble() * 1.0;
                        int randInt = Math.abs(random.nextInt());
                        MicroModel modelNew = (rand < perTr)
                                ? truckmodel : carmodel;
                        LaneChange changemodelNew = (rand < perTr)
                                ? polite : inconsiderate;
                        double vNew = modelNew.Veq(space);
                        if (false) {
                            System.out.println("MicroStreet.ioFlow: "
                                    + ((rand < perTr) ? "Truck" : "Car")
                                    + ", vNew=" + vNew);
                        }
                        double lNew = (rand < perTr) ? LKW_LENGTH_M : PKW_LENGTH_M;
                        Color colorNew = (rand < perTr)
                                ? colorTruck : colorCar;

                        imax = street.size();
                        /*street.insertElementAt(new Car(0.0, vNew, lane, modelNew, changemodelNew,
                                lNew, colorNew, randInt), imax);*/
                    }
                } // at least one lane without vehicles
                else {
                    MicroModel carmodel = (choice_Szen != 4) ? idmCar : sync1Car;
                    MicroModel truckmodel = (choice_Szen != 4) ? idmTruck : sync1Truck;
                    double rand = random.nextDouble() * 1.0;
                    int randInt = Math.abs(random.nextInt());
                    MicroModel modelNew = (rand < perTr) ? truckmodel : carmodel;
                    double vNew = modelNew.Veq(spaceFree);
                    double lNew = (rand < perTr) ? LKW_LENGTH_M : PKW_LENGTH_M;
                    Color colorNew = (rand < perTr) ? colorTruck : colorCar;
                    int lane = (i_lu < 0) ? 0 : 1;
                    imax = street.size();
                    //System.out.println("street.size()=" + street.size());
                    /*street.insertElementAt(new Car(0.0, vNew, lane, modelNew,
                            inconsiderate, lNew, colorNew, randInt), imax);*/
                }
            }
        }
        //      System.out.println("ioFlow end: imax="+(street.size()-1));

    }

    // Sort to decreasing values of pos using the bubblesort algorithm;
    // Pairwise swaps running over all vehicles; 
    // repeat loop over vehicles until
    // sorted; typically, only 2 runs over the loop are needed
    // (one to sort; one to check)
    protected void sort() {

        boolean sorted = false;

        while (!sorted) {

            sorted = true;
            int imax = street.size();
            for (int i = 1; i < imax; i++) {
                double p_back = ((Moveable) street.elementAt(i)).position();
                double p_front = ((Moveable) street.elementAt(i - 1)).position();
                if (p_back > p_front) {
                    sorted = false;
                    Moveable temp = (Moveable) street.elementAt(i - 1);
                    street.setElementAt((Moveable) street.elementAt(i), i - 1);
                    street.setElementAt(temp, (i));
                }
            }
        }
    }

    // returns index of first (most downstream) vehicle on given lane;
    // if no vehicles on this lane; -1 is returned
    protected int firstIndexOnLane(int lane) {
        int nr_max = (street.size()) - 1;
        //System.out.println(" tamaño de street en firstIndex = " + nr_max);
        int i = 0;
        boolean carFound = false;
        if (nr_max >= 0) {
            while ((i <= nr_max) && (!carFound)) {
                if (((Moveable) street.elementAt(i)).lane() == lane) {
                    //  if (((((Moveable)street.elementAt(i)).lane())==lane)&&(flag==0)){
                    carFound = true;
                }
                //System.out.println 
                //("first-loop:"+(new Integer(i).toString()));
                i++;
            }
        }
        return ((carFound) ? i - 1 : -1);
    }

    // returns index of most upstream vehicle on given lane
    protected int lastIndexOnLane(int lane) {

        int nr_max = (street.size()) - 1;
        //System.out.println(" tamaño de street en lastIndex = " + nr_max);
        int i = nr_max;
        boolean carFound = false;
        if (nr_max >= 0) {
            while ((i >= 0) && (!carFound)) {
                if (((Moveable) street.elementAt(i)).lane() == lane) {
                    //  if (((((Moveable)street.elementAt(i)).lane())==lane)&&(flag==0)){
                    carFound = true;
                }
                i--;
            }
        }
        return ((carFound) ? i + 1 : -1);
    }

    // !! bounds not checked
    protected int nextIndexOnLane(int lane, int ind) {
        //textarea.setText("In nextIndexOnLane");
        int next_ind = ind - 1;
        while ((((Moveable) street.elementAt(next_ind)).lane()) != lane) {
            next_ind--;
        }
        return next_ind;
    }

    // !! bounds not checked
    protected int prevIndexOnLane(int lane, int ind) {
        //textarea.setText("In nextIndexOnLane");
        int next_ind = ind + 1;
        while ((((Moveable) street.elementAt(next_ind)).lane()) != lane) {
            next_ind++;
        }
        return next_ind;
    }

    // !! assumed that neighbours are existent; otherwise OutOfBoundsException
    protected void setNeighbours(int ind) {
        int vl = nextIndexOnLane(0, ind);
        int vr = nextIndexOnLane(1, ind);
        int hl = prevIndexOnLane(0, ind);
        int hr = prevIndexOnLane(1, ind);

        vL = (Moveable) street.elementAt(vl);
        vR = (Moveable) street.elementAt(vr);
        hL = (Moveable) street.elementAt(hl);
        hR = (Moveable) street.elementAt(hr);
    }

    // remove the two white obstacles when traffic ight turns green
    // (only once in a simulation and only for choice_Szen==5)
    protected void open() {
        //street.removeElementAt(0);
        street.removeElementAt(0);
    }
    public void outFlow(double time) {
        double eTime=time;
        int imax = street.size()-2;
        //System.out.println(" en outFlow: "+ imax);
        if (imax >= 1) {
             //System.out.println("MicroStreet.oFlow: removing vehicle...");
            while ((imax >= 1) && // (imax>=0) first to prevent indexOutOf...!
                    (((Moveable) (street.elementAt(1))).position() >= roadLength-LKW_LENGTH_M)) {
                //timeS = ((Moveable) (street.elementAt(0))).exitTime();//tiempo en el sistema
                int i=exit.size();
                exit.insertElementAt(new Double(eTime), i);// alamcena el tiempo de salida
                //Moveable exit = ((Moveable) (street.elementAt(0)));
                //exit.setExitTime(time);
                double aTime = ((Moveable) (street.elementAt(1))).arriveTime();
                double sTime = eTime - aTime;//Tiempo de servicio
                //numVeh ++;
                System.out.println(" Llegada "+ aTime+" Salida "+ eTime+" Servicio "+sTime);
                //sumTimes = sumTimes + sTime;
                //serviceT = sumTimes/numVeh;// promedio tiempo de servicio
                //serviceT = sTime;
                street.removeElementAt(1);
                imax--;
            }
        }
    }
    
    //Imprime los tiempos de llegada de cada vehiculo al sistema
    public void timesArrive(){
        double eTime=time;
        int imax = street.size();
        int i=0;
        double time;
        for( i = 0; i< imax; i++){
            
            time = ((Moveable) (street.elementAt(i))).arriveTime(); 
            System.out.println(" time arrive" + "["+i+"] = "+ time);
            double aTime = ((Moveable) (street.elementAt(i))).arriveTime();
            double sTime = eTime - aTime;
            System.out.println(" Llegada "+ aTime+" Salida "+ eTime+" Servicio "+sTime);
            
            int j= arrive.size();
            arrive.insertElementAt(time, j);
        }
    }
    
   
    
    
              
    //para el calculo del tiempo de servicio en el sistema
    public void calcTimesService(){
        
        int exits = exit.size();
        int i;
        double eTime, aTime,sTime;// tiempo de llegada, salida y servicio
        if(exits >= 0){
            for( i = 0; i< exits; i++){
            
                aTime =(Double)arrive.elementAt(i); //llegada
                eTime =(Double)exit.elementAt(i); //salida
                sTime = eTime - aTime; 
                System.out.println("-----------------Vehiculo ["+i+"]: Llegada "+ aTime+" Salida "+ eTime+" Servicio "+sTime);
            }
        }
        
            
    }
}
