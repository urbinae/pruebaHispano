/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

import java.awt.Color;

/**
 *
 * @author eimar
 */
public class FromValle extends MicroStreet {

    private double laneLength;// tamaño del canal desde el Valle hasta la interseccion   
    private ToMerida toMerida;
    private SimCanvas simcanvas;
    private Moveable me, newMe;
    private Moveable frontVeh;
    private Moveable frontVehTM;
    private Moveable backVehTM;
    private int i_insertTM;
    private LaneChange laneChangeModel;

    public FromValle(SimCanvas simcanvas, double laneLength, double mergingPosAtToMerida, double p_factor, double deltaB, int choice_Szen) {
        super(laneLength, 0., p_factor, deltaB, 0, choice_Szen);

        if (choice_Szen == 7 || choice_Szen == 9) {
            laneChangeModel = new LaneChange(p_factor, deltaB);
        }

        this.laneLength = laneLength;

        this.toMerida = simcanvas.toMerida;

        this.simcanvas = simcanvas;
        interFV = roadLength - 2 * simcanvas.rd_Merida;

        if (choice_Szen == 8 || choice_Szen == 10) {
            street.addElement(new Obstacle(roadLength + 100, 0, 0., 0));
        } else {
            street.addElement(new Obstacle(roadLength, 0, 0., 0));
        }

    }

    public void updateFromValle(double time, double dt, double qIn, double perTr,
            double p_factor, double deltaB, int choice_Szen) {

        this.time = time;

        if (choice_Szen == 7 || choice_Szen == 9) {
            laneChangeModel.set_p(p_factor);
            laneChangeModel.set_db(deltaB);

        }
        old_pos = setPos();        // need old info for detectors and drawing

        accelerate(dt);
        translate(dt);

        positions = setPos();
        velocities = setVel();
        colors = setColors();
        lengths = setLengths();

        inFlowFromValle(this.time, dt, qIn, perTr);
        if (choice_Szen == 8 || choice_Szen == 10) {
            outFlow(time);
        } else {
            mergeToMerida(laneChangeModel, choice_Szen);
        }



    }

    // para el calculo de la aceleracion
    protected void accelerate(double dt) {

        int imax = street.size() - 1;
        for (int i = imax; i >= 1; i--) {
            // Se obtiene mi auto y el del frente en la rampa
            Moveable me = (Moveable) street.elementAt(i);
            Moveable frontVeh = (Moveable) street.elementAt(i - 1);

            //Llamada al metodo accelerate de la interfaz Moveable implementado en la clase Car
            me.accelerate(dt, frontVeh);
        }
    }

    protected int translate(double dt) {

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

            if (choice_Szen == 8 || choice_Szen == 10) {
                if ((x_old <= interFV) && (x_new > interFV)) {
                    if (me.model() == idmTruckFV) {
                        me.setModel(syncTruckTT);
                    } else {
                        me.setModel(syncCarTT);
                    }
                }
            }
        }
        return street.size();
    }

    //Flujo de entrada
    private void inFlowFromValle(double time, double dt, double qIn, double perTr) {
        double arriveT = time;
        System.out.println(" En inFlowFromValle: laneLength= " + laneLength);
        final double spaceMin = 10.; // minimum headway for new vehicle
        int lane = 1;
        double space = 0;

        nin = nin + qIn * dt;// = 0+0,1*0,25= 0,111
        //System.out.println(" FromValle: nin = " + nin);
        if (nin > 1.0) {  // new vehicle imposed by the inflow BC, si num veh in>1 
            nin -= 1.0; // le resto 1 a nin
            int iPrev = street.size() - 1;// indice del veh previo
            if (iPrev >= 0) { // si 4>0
                space = ((Moveable) (street.elementAt(iPrev))).position();// posicion del vehiculo de enfrente
                //      System.out.println(" FromValle: iprev = " + iPrev + ", space = " + space + ", spaceMin = " + spaceMin);
            } // si no
            else {
                space = spaceMin + 0.1; // la posicion de ese vehiculo es mayor que el espacio min
                //    System.out.println(" FromValle: iprev = " + iPrev + ", space = " + space + ", spaceMin = " + spaceMin);
            }

            // ¿El espacio es suficiente para el nuevo vehiculo que ingresa?
            if (!(red = (space < spaceMin))) {// !(bandera 

                double rand = random.nextDouble() * 1.0;// numero aleatorio (double) entre 0y1
                int randInt = Math.abs(random.nextInt());// numero aleatorio (int) positivo
                MicroModel modelNew = (rand < perTr) ? idmTruckFV : idmCarFV; // Micromodel para la velocidad y la aceleracion
                double vNew = modelNew.Veq(space);// asigna la velocidad
                double lNew = (rand < perTr) ? LKW_LENGTH_M : PKW_LENGTH_M; // asigna el tamaño del vehiculo
                Color colorNew = (rand < perTr) ? colorTruck : colorCarTV; // asigna el color, rojo o negro
                int origin = ORIGINFV;
                street.insertElementAt// inserta el vehiculo en la posicion street.size()
                        ((new Car(0.0, vNew, lane, origin, modelNew, inconsiderate, arriveT,
                        lNew)), street.size());

                //  System.out.println(" inFlowFromValle: " + street.size());
            }
        }

    }

    private void mergeToMerida(LaneChange laneChangeModel, int choice_Szen) {
        // Si se escoge el escenario 1, se desplega la situacion actual
        if (choice_Szen == 7 || choice_Szen == 9) {
            int imax = street.size() - 1;
            //System.out.println("in mergeToTMRoad: street= "+imax);
            int nvehTM = simcanvas.toMerida.street.size();
            //System.out.println("in mergeToTMRoad: TMroad.street= "+nvehTM);

            if (false) {
                System.out.println("in mergeToMerida: "
                        + "1 obstacle + " + (imax) + " veh(s) on From Valle, "
                        + nvehTM + " on To Merida;");
                //+" offset="+(int)(offsetTM));
            }

            if (imax >= 1) {  // at least one real vehicle on the ramp lane
                //for (int i=1; i<=imax; i++){ 
                me = (Car) (street.elementAt(1));
                double x = me.position();
                double gapFV = laneLength - x - me.length();
                //System.out.println("in mergeToTMRoad: mi posicion"+x);
                //System.out.println("in mergeToTMRoad: roadLength "+roadLength+" rampLength"+rampLength);
                //double dif = roadLength-rampLength;
                //System.out.println("in mergeToTMRoad: roadLength - rampLength= "+dif);

                if ((gapFV >= 0 && gapFV <= 5)) { // action only in merging region

                    System.out.println(" mergeToTMRoad: veh in intersection! " + x);
                    x = 15;
                    //me.setPosition(x);
                    System.out.println(" mergeToTMRoad: new position in intersection! " + x);

                    frontVeh = (Moveable) street.elementAt(0);
                    setNeighboursOnTMRoad(x);
                    // -> virtual vehicles frontVehTM, backVehTMfRamp, 
                    // position i_insertTM
                    //  positions are given in the ramp system!

                    // do actual change if incentive criterion fulfilled
                    me.setLaneChange(laneChangeModel); // update lane-change params
                    double v = me.velocity();
                    MicroModel model = me.model();
                    double l = me.length();
                    double t = me.arriveTime();
                    newMe = new Car(x, v, 1, 3, model, inconsiderate,t, l);
                    newMe.setLaneChange(laneChangeModel);
                    if (newMe.change(frontVeh, frontVehTM, backVehTM)) {

                        System.out.println(" mergeToTMRoad: Changing!!!\n "
                                + "  deltaB=" + ((Car) me).lanechange.get_db()
                                + "  nvehTM=" + simcanvas.toMerida.street.size()
                                + "  real veh From Valle=" + (street.size() - 1));

                        //me.setLane(1); // right lane on future TM road
                        me.setPosition(x);
                        me.setColor(Color.black);
                       
                        me.setLaneChange(simcanvas.toMerida.inconsiderate);
                        simcanvas.toMerida.street.insertElementAt(me, i_insertTM);
                        street.removeElementAt(1);
                        imax--;

                        System.out.println("  Changed!!! "
                                + " nvehTM=" + simcanvas.toMerida.street.size()
                                + " i_insertTM = " + i_insertTM
                                + " pos " + me.position()
                                + " real veh From Valle=" + imax);
                    }
                }//fin if x in region
                //}fin for
            }

        }
        if (choice_Szen == 8 || choice_Szen == 10) {
            outFlow(this.time);
        }
    }

    private void setNeighboursOnTMRoad(double x) {

        // -> virtual vehicles frontVehTM, backVehTMfRamp
        // whose positions are given in the ramp system, and index
        // i_insertTM at which car is considered to be placed on TM road

        //boolean debug=(i==1);
        boolean debug = true;
        //double spaceMin = 10.;
        final double farDistance = 10000;

        int nvehTM = simcanvas.toMerida.street.size();
        int iTM = 0;
        double xTM = farDistance;
        //int    laneTM;
        int i_frontTM;
        int i_backTM;

        if (nvehTM > 0) {
            // System.out.println(" setNeighboursOnTMRoad: "
            //   + "there are nvehTM="+nvehTM+" > 0 TM vehicles!");


            // determine index of last vehicle in front of ramp vehicle
            // on TM road (regardless of lane!)

            System.out.println("On From Valle: setNeighbours..antes del for num veh hacia merida " + nvehTM);
            for (iTM = 0; ((iTM < nvehTM) && (x < xTM)); iTM++) {
                xTM = ((Moveable) (simcanvas.toMerida.street.elementAt(iTM))).position();

                if (debug) {
                    System.out.println("On From Valle: setNeighbours..for loop: iTM= " + iTM + " x= " + x
                            + " xTM= " + ((int) (xTM)) + " nvehTM= " + nvehTM);
                }
            }
            if (xTM > x) {                
                i_frontTM = iTM-1; //!!! iTM-2
                i_backTM = nvehTM +1;
                i_insertTM = iTM-1; //!!! iTM-1
            } else {

                i_frontTM = iTM - 2; //!!! iTM-2
                i_backTM = iTM-1;   //!!! iTM-1
                i_insertTM = iTM-1; //!!! iTM-1
            }

            if (debug) {
                System.out.println("On From Valle:setNeighbours..: i from Valle= " + 1
                        + "\ni_frontTM= " + i_frontTM
                        + "\ni_backmTM= " + i_backTM
                        + "\ni_insertTM= " + i_insertTM);
            }


            //  i_frontTM--;

            if (debug) {
                if (i_frontTM == -1) {
                    System.out.println("From Valle:setNeighbours: No front vehicle, but >=1 back veh! ");
                }
                if (i_frontTM + 2 == nvehTM) {
                    System.out.println("From Valle:setNeighbours: No back vehicle, but >=1 front veh! ");
                }
            }
        } else { // nvehTM=0
            if (debug) {
                System.out.println("On From Valle:setNeighbours: nvehTM= "
                        + nvehTM + " => no vehicle on to Merida! ");
            }
            i_frontTM = -1;
            i_backTM = -1;
            i_insertTM = iTM;
        }

        // define virtual cars.
        // must copy cars (new ...) because otherwise (pointer assignment)
        // offset action below would offset streets on TMroad!
        
        //double t = ((Moveable) (simcanvas.toMerida.street.elementAt(i_frontTM))).arriveTime();
        frontVehTM = (i_frontTM < 0) // only back vehicle(s)
                ? new Car(farDistance, 0, 1, 3,
                idmCarFV, inconsiderate,0, 5)
                : new Car((Car) (simcanvas.toMerida.street.elementAt(i_frontTM)));


        backVehTM = ((nvehTM < 1) || (i_backTM >= nvehTM))
                //? new Car(-farDistance, 0, 0, 3, // only front veh(s) or none
                //idmCarFV, inconsiderate, 5)
                ? new Car(-farDistance, 0, 1, 3, idmCarFV, inconsiderate, 0, 5)
                : new Car((Car) (simcanvas.toMerida.street.elementAt(i_backTM)));


        if (debug) {
            System.out.println(" setNeighboursOnTMRoad!!!: "
                    + " nvehTM= " + nvehTM
                    + " i_backTM= " + i_backTM
                    + " x_back= " + backVehTM.position());
        }
        // adjust positions to ramp system

        frontVehTM.setPosition(frontVehTM.position());
        backVehTM.setPosition(backVehTM.position());

        if (true) {
            System.out.println(" setNeighboursOnTM, got neighbours: "
                    + "\n   i_frontTM= " + i_frontTM
                    + ", x_front= " + frontVehTM.position()
                    + ", x_back= " + backVehTM.position());
        }
    } // end setNeighboursOnTMRoad

    public void outFlow(double time) {
        double eTime = time;
        int imax = street.size() - 2;
        //System.out.println(" en outFlow: "+ imax);
        if (imax >= 1) {
            //System.out.println("MicroStreet.oFlow: removing vehicle...");
            while ((imax >= 1) && // (imax>=0) first to prevent indexOutOf...!
                    (((Moveable) (street.elementAt(1))).position() >= roadLength)) {
                //timeS = ((Moveable) (street.elementAt(0))).exitTime();//tiempo en el sistema
                int i = exit.size();
                exit.insertElementAt(new Double(eTime), i);// alamcena el tiempo de salida
                //Moveable exit = ((Moveable) (street.elementAt(0)));
                //exit.setExitTime(time);
                double aTime = ((Moveable) (street.elementAt(1))).arriveTime();
                double sTime = eTime - aTime;//Tiempo de servicio
                numVehFV++;
                System.out.println(" Llegada " + aTime + " Salida " + eTime + " Servicio " + sTime);
                sumTimesFV = sumTimesFV + sTime;
                serviceTFV = sumTimesFV / numVehFV;// promedio tiempo de servicio
                //serviceT = sTime;
                street.removeElementAt(1);
                imax--;
            }
        }
    }
}
