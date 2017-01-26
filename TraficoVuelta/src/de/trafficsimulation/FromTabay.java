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
public class FromTabay extends MicroStreet {

    private double laneLength;// tamaño del canal desde la redoma hasta la interseccion
    private ToValle toValle;// para actualizar los vectores de la via hacia el Valle
    private ToTabay toTabay;// para actualizar los vectores de la via hacia Tabay    
    private ToMerida toMerida;
    private FromValle fromValle;
    private FromMerida fromMerida;
    private Moveable me;// yo el canal desde Merida
    private Moveable lastToTabay; // ultimo vehiculo en el canal hacia Tabay
    private Moveable firstFromMerida;// primer vehiculo en desde Tabay
    private Moveable lastToMerida;
    private Moveable lastToValle;
    private SimCanvas simcanvas;
    private LaneChange laneChangeModel;

    public FromTabay(SimCanvas simcanvas, double laneLength, double mergingPos, double p_factor, double deltaB, int choice_Szen) {

        super(laneLength, 0., p_factor, deltaB, 0, choice_Szen);

        laneChangeModel = new LaneChange(p_factor, deltaB);
        this.laneLength = laneLength;
        this.fromMerida = simcanvas.fromMerida;
        this.fromValle = simcanvas.fromValle;
        this.toMerida = simcanvas.toMerida;
        this.toValle = simcanvas.toValle;
        this.toTabay = simcanvas.toTabay;
        this.simcanvas = simcanvas;

        upcurve1 = (STRAIGHT3_M + Math.PI * RADIUS_M + STRAIGHT2_M) - 20.;
        upcurve2 = STRAIGHT3_M - 20;
        cercaInter = STRAIGHT3_M + Math.PI * RADIUS_M + STRAIGHT2_M + Math.PI * RADIUS_M + STRAIGHT1_M / 2;


        this.laneLength = laneLength;

        street.addElement(new Obstacle(roadLength, 0, 0., 0));
        //street.addElement(new Obstacle(roadLength - 10., 0, 1., 0));
        //street.addElement(new Car(0.0, idmCarFT.Veq(roadLength), 0, idmCarFT, inconsiderate,
        //      PKW_LENGTH_M, colorCarFM, 2));

    }

// Actualiza el canal (desdeMerida)
    public void updateFromTabay(double time, double dt, double qIn, double perTr, double p_factor, double deltaB, int prior) {

        this.time = time;
        laneChangeModel.set_p(p_factor);
        laneChangeModel.set_db(deltaB);
        // para actualizar las viejas posiciones
        //System.out.println("--------------fromMerida: old pos---------------");
        old_pos = setPos();        // need old info for detectors and drawing

        accelerate(dt); // calcula la aceleracion
        translate(dt);// translada el vehiculo una posicion adelante

        //System.out.println("--------------fromMerida: new pos---------------");
        positions = setPos();
        velocities = setVel();
        colors = setColors();
        lengths = setLengths();

        inFlowFromTabay(time, dt, qIn, perTr, prior);
        if (choice_Szen == 7 || choice_Szen == 8) {
            intersectionCheck();
        }
        if (choice_Szen == 9 || choice_Szen == 10) {
            intersectionCheck2();
        }

    }
    // para el calculo de la aceleracion

    protected void accelerate(double dt) {

        int imax = street.size() - 1;
        for (int i = imax; i >= 1; i--) {
            // Se obtiene mi vehiculo y el de en frente de mi
            Moveable me = (Moveable) street.elementAt(i);
            Moveable frontVeh = (Moveable) street.elementAt(i - 1);

            //Llamada al metodo accelerate de la interfaz Moveable implementado en la clase Car
            me.accelerate(dt, frontVeh);
        }
    }
//Metodo que me permite verificar la proxima posicion para reducir la velocidad en cirto punto
// y crear cuello de botella

    private int translate(double dt) {

        // without sorting

        int imax = street.size();
        for (int i = 0; i < imax; i++) {
            Moveable me = (Moveable) street.elementAt(i);
            double x_old = me.position();
            //System.out.println(" En FromTabay Velocidad:= "+me.velocity());
            me.translate(dt);
            double x_new = me.position();

            // if Szenario 4, flow-conserving bottlenecks 
            // rea7lized with parameter
            // gradients and uppos is crossed: 
            // Change upstream to downstream parameters


            if ((x_old <= upcurve2) && (x_new > upcurve2)) {
                //System.out.println(" En FromTabay Curva 2 Velocidad:= "+me.velocity());
                if (me.model() == idmTruckFT) {
                    me.setModel(syncTruckFT);
                } else {
                    me.setModel(syncCarFT);
                }
            }

            if ((x_old <= upcurve1) && (x_new > upcurve1)) {
                //System.out.println(" En FromTabay Curva 1 Velocidad:= "+me.velocity());
                if (me.model() == syncTruckFT) {
                    //me.setModel(idmTruckFT);
                    me.setModel(syncTruckFT);
                } else {
                    //me.setModel(idmCarFT);
                    me.setModel(syncCarFT);
                }
            }

            if ((x_old <= cercaInter) && (x_new > cercaInter)) {
                //System.out.println(" En FromTabay Curva 2 Velocidad:= "+me.velocity());
                if (me.model() == syncTruckFT) {
                    me.setModel(syncTruckFT);
                } else {
                    me.setModel(syncCarFT);
                }
            }

        }
        return street.size();
    }

// para el flujo de entrada del canal desde Tabay hacia Merida 
    // con lane = 0
    private void inFlowFromTabay(double time, double dt, double qIn, double perTr, int prior) {
        System.out.println(" En inFlowfromTabay: laneLength= " + laneLength);
        double arriveT=time;
        final double spaceMin = 10; // minimum headway for new vehicle
        int lane = 1; // canal desdeTabay hacia Merida
        double space = 0;// posicion del nuevo vehiculo: si es menor que spaceMin, si se inserta

        nin = nin + qIn * dt;// = 0+0,1*0,25= 0,111
        //System.out.println(" fromMerida: nin = " + nin);
        if (nin > 1.0) {  // new vehicle imposed by the inflow BC, si num veh in>1 
            nin -= 1.0; // le resto 1 a nin
            int iPrev = street.size() - 1;// indice del veh previo, ejemp: 5-1=4
            if (iPrev >= 0) { // si 4>0
                space = ((Moveable) (street.elementAt(iPrev))).position();// posicion del vehiculo de enfrente
                //System.out.println(" fromMerida: iprev = " + iPrev + ", space = " + space + ", spaceMin = " + spaceMin);
            } // si no
            else {
                space = spaceMin + 0.1; // la posicion de ese vehiculo es mayor que el espacio min
                //System.out.println(" fromMerida: iprev = " + iPrev + ", space = " + space + ", spaceMin = " + spaceMin);
            }


            // enough space for new vehicle to enter? (!red)
            // ¿El espacio es suficiente para el nuevo vehiculo que ingresa?
            if (!(red = (space < spaceMin))) {// !(bandera = (20<5)), entonces, se inserta
                /*MicroModel carmodel = (choice_Szen != 4)
                ? idmCar : sync1Car;
                MicroModel truckmodel = (choice_Szen != 4)
                ? idmTruck : sync1Truck;*/

                double rand = random.nextDouble() * 1.0;// numero aleatorio (double) entre 0y1
                //int randInt = Math.abs(random.nextInt());// numero aleatorio (int) positivo
                MicroModel modelNew = (rand < perTr) ? idmTruckFT : idmCarFT; // Micromodel para la velocidad y la aceleracion
                double vNew = modelNew.Veq(30.);// asigna la velocidad aprox. de 40 Km/h 
                System.out.println(" En inFlowfromTabay: Velocidad de entrada= " + vNew);
                double lNew = (rand < perTr) ? LKW_LENGTH_M : PKW_LENGTH_M; // asigna el tamaño del vehiculo
                Color colorNew = (rand < perTr) ? colorTruck : colorCar; // asigna el color, rojo o negro
                //int priority = (rand < PRIORFT) ? 1 : 0; // prioridad para cruzar de los vehiculos FromTabay
                int origin = ORIGINFT;
                //  estos tienen mayor prioridad de cruzar que los de FromMerida
                int prioridad;
                if (contPrior == prior) {
                    System.out.println(" En inFlowFromTabay: volver a cero el contador de prioridades, contPrior = " + contPrior);
                    prioridad = 0;
                    contPrior = 0;
                } else {
                    System.out.println(" En inFlowFromTabay: contPrior = " + contPrior);
                    prioridad = 1;
                    System.out.println(" En inFlowFromTabay: Prioridad = " + prioridad);
                    contPrior++;
                }
                street.insertElementAt// inserta el vehiculo en la posicion street.size()
                        ((new Car(0.0, vNew, lane, origin, prioridad, inconsiderate, modelNew, arriveT,
                        lNew)), street.size());
            }
        }

    }
//Chequea la interseccion desde Tabay

    private void intersectionCheck() {
        final double spaceMin = 15; // tamaño de un camion = 10., mas brecha minima = 2
        //double gap = 10.;
        int nvehFromTabay = street.size() - 1; // nuemro de vehiculos en el canal desde Merida
        //System.out.println("in intersecctionCheck FromTabay: desde Tabay= " + nvehFromTabay);

        //int nvehFromValle = simcanvas.fromValle.street.size() - 1;// numero de vehiculos en el canal hacia Merida
        //System.out.println("in intersecctionCheck FromTabay:  desde el Valle= " + nvehFromValle);

        int nvehFromMerida = simcanvas.fromMerida.street.size() - 1;// numero de vehiculos en el canal desde Tabay
        //System.out.println("in intersecctionCheck FromTabay:  desde Merida= " + nvehFromMerida);

        int nvehToValle = simcanvas.toValle.street.size();// numero de vehiculos en el canal hacia el Valle
        //System.out.println("in intersecctionCheck FromTabay:  hacia el Valle= " + nvehToValle);

        //int nvehToTabay = simcanvas.toTabay.street.size();// numero de vehiculos en el canal hacia Tabay
        //System.out.println("in intersecctionCheck FromTabay:  hacia Tabay= " + nvehToTabay);

        int nvehToMerida = simcanvas.toMerida.street.size();// numero de vehiculos en el canal hacia Merida
        //System.out.println("in intersecctionCheck FromTabay:  hacia Merida= " + nvehToMerida);

        //System.out.println(" in intersectionCheck FromTabay: Tamaño del canal de Tabay " + roadLength);
        //System.out.println(" in intersectionCheck FromTabay: Tamaño del canal de Merida " + simcanvas.fromMerida.roadLength);
        //System.out.println(" in intersectionCheck FromTabay: Tamaño del canal de El Valle " + simcanvas.fromValle.roadLength);


        // Vehiculos en el canal desde Tabay y cero en el canal desde Merida
        if ((nvehFromTabay >= 1 && nvehFromMerida < 1)) {
            //  System.out.println(" in intersecctionCheck FromTabay: (nvehFromTabay >= 1 && nvehFromMerida < 1)");
            me = (Car) (street.elementAt(1)); // asigno a 'me' el elemento (1) (este es el vehiculo que esta en la posicion mas cerca de la interseccion)
            double x = me.position();// obtengo mi posicion
            //System.out.println(" in intersecctionCheck FromTabay: FT posicion del vehiculo mas cercano a la inter: " + x);

            double gapFT = laneLength - x - me.length();
            if (gapFT >= 0 && gapFT <= 5) {
                if (nvehToValle >= 1) {
                    lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                    double posTV = lastToValle.position();
                    if (posTV >= spaceMin) {
                        if (nvehToMerida >= 1) {
                            lastToMerida = (Moveable) (simcanvas.toMerida.street.elementAt(nvehToMerida - 1));
                            //      System.out.println(" in intersecctionCheck FromTabay: 1 Cambio de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                        } else {
                            //    System.out.println(" in intersecctionCheck FromTabay: 2 Cambio de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                        }
                    }
                } else {
                    if (nvehToMerida >= 1) {
                        lastToMerida = (Moveable) (simcanvas.toMerida.street.elementAt(nvehToMerida - 1));
                        //      System.out.println(" in intersecctionCheck FromTabay: 1 Cambio de Tabay a Merida");
                        changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                    } else {
                        //    System.out.println(" in intersecctionCheck FromTabay: 2 Cambio de Tabay a Merida");
                        changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                    }
                }
            }
        }//fin si solo veh desde Tabay


        // Vehiculo(s) en el canal desde Tabay y en el canal desde Merida 
        if (nvehFromTabay >= 1 && nvehFromMerida >= 1) {
            System.out.println(" in intersecctionCheck FromTabay: (nvehFromTabay >= 1 && nvehFromMerida >= 1)");
            // primer vehiculo desde Tabay (mas cercano a la interseccion)
            me = (Moveable) (street.elementAt(1)); // 
            double x = me.position();// obtengo mi posicion
            //System.out.println(" in intersecctionCheck FromTabay: posicion del vehiculo mas cercano a la inter: " + x);

            // primer vehiculo desde Merida (mas cercano a la interseccion)
            firstFromMerida = (Moveable) (simcanvas.fromMerida.street.elementAt(1));
            double posFM = firstFromMerida.position();

            //System.out.println(" in intersecctionCheck FromTabay: FM posFM " + posFM);

            //double gapFT = laneLength - x - me.length() - ((Obstacle) street.elementAt(0)).length();
            double gapFT = laneLength - x - me.length();
            double gapFM = simcanvas.fromMerida.length() - posFM - firstFromMerida.length();
            //System.out.println(" in intersectionCheck FromTabay: brecha FM " + gapFM + " brecha FT" + gapFT);
            if ((gapFM >= 0 && gapFM <= 5) && (gapFT >= 0 && gapFT <= 5)) { // si los vehiculos estan en la interseccion
                System.out.println(" in intersecctionCheck FromTabay: Prioridades " + me.priority() + " && " + firstFromMerida.priority());

                //  System.out.println(" in intersecctionCheck FromTabay: ((gapFM >= 0 && gapFM <= 1) && (gapFT >= 0 && gapFT <= 1)) ");
                //System.out.println(" in intersecctionCheck FromTabay: gapFM " + gapFM + " gapFT " + gapFT);

                if (me.priority() == 1 && firstFromMerida.priority() == 0) {// el veh desde merida Va hacia el valle
                    // System.out.println(" in intersecctionCheck FromTabay: me.destino()= " + firstFromMerida.destino() + " && me.priority()= " + me.priority());
                    if (nvehToValle >= 1) {
                        lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                        double posTV = lastToValle.position();
                        if (posTV >= spaceMin) {
                            if (nvehToMerida >= 1) {
                                lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                                //            System.out.println(" in intersecctionCheck FromTabay: 3 Cambiar de Tabay hacia Merida");
                                changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);// me=from, lastToValle=to                 
                            } else {
                                //          System.out.println(" in intersecctionCheck FromTabay: 4 Cambio de Tabay a Merida");
                                changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);// me=from, lastToValle=to                 
                            }
                        }
                    } else {
                        if (nvehToMerida >= 1) {
                            lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                            //            System.out.println(" in intersecctionCheck FromTabay: 3 Cambiar de Tabay hacia Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);// me=from, lastToValle=to                 
                        } else {
                            //          System.out.println(" in intersecctionCheck FromTabay: 4 Cambio de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);// me=from, lastToValle=to                 
                        }
                    }
                } else if ((me.priority() == 0 && firstFromMerida.priority() == 0)){
                    System.out.println(" in intersecctionCheck FromTabay: Prioridades iguales " + me.priority() + " && " + firstFromMerida.priority());
                    me.setPriority(1);
                    //firstFromMerida.setPriority(1);
                    System.out.println(" in intersecctionCheck FromTabay: Nuevas Prioridades  " + me.priority() + " && " + firstFromMerida.priority());

                }
                if ((me.priority() == 1 && firstFromMerida.priority() == 1)){
                    System.out.println(" in intersecctionCheck FromTabay2: Prioridades iguales " + me.priority() + " && " + firstFromMerida.priority());
                    //me.setPriority(1);
                    firstFromMerida.setPriority(0);
                    System.out.println(" in intersecctionCheck FromTabay2: Nuevas Prioridades  " + me.priority() + " && " + firstFromMerida.priority());

                }
            } else {
                if ((gapFM > 5) && (gapFT >= 0 && gapFT <= 5)) {
                    //if (me.priority() == 0 || me.priority() == 1) {
                    //System.out.println(" in intersecctionCheck FromTabay: me.destino()= " + firstFromMerida.destino() + " && me.priority()= " + me.priority());
                    //System.out.println(" in intersecctionCheck FromTabay: ((gapFM > 1) && (gapFT>=0 && gapFT <= 1))");
                    if (nvehToValle >= 1) {
                        lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                        double posTV = lastToValle.position();
                        if (posTV >= spaceMin) {
                            if (nvehToMerida >= 1) {
                                lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                                //          System.out.println(" in intersecctionCheck FromTabay: 5 Cambiar de Tabay a Merida");
                                changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                            } else {
                                //        System.out.println(" in intersecctionCheck FromTabay: 6 Cambiar de Tabay a Merida");
                                changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                            }
                        }
                    } else if (nvehToValle < 1) {
                        if (nvehToMerida >= 1) {
                            lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);

                            //  System.out.println(" in intersecctionCheck FromTabay: 7 Cambiar de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                        } else {
                            //System.out.println(" in intersecctionCheck FromTabay: 8 Cambiar de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                        }
                    }
                    //}
                }
            }

        }// else, no hay vehiculos en ambos canales
    }

    // Para el escenario ampliacion de From Merida
    private void intersectionCheck2() {
        final double spaceMin = 15; // tamaño de un camion = 10., mas brecha minima = 2
    
        int nvehFromTabay = street.size() - 1; // nuemro de vehiculos en el canal desde Merida
     
        int nvehFromMerida = simcanvas.fromMerida.street.size() - 1;// numero de vehiculos en el canal desde Tabay
    
        int nvehToValle = simcanvas.toValle.street.size();// numero de vehiculos en el canal hacia el Valle
       
        int nvehToMerida = simcanvas.toMerida.street.size();// numero de vehiculos en el canal hacia Merida
       
      

        // Vehiculos en el canal desde Tabay y cero en el canal desde Merida
        if ((nvehFromTabay >= 1 && nvehFromMerida < 1)) {
            //  System.out.println(" in intersecctionCheck FromTabay: (nvehFromTabay >= 1 && nvehFromMerida < 1)");
            me = (Car) (street.elementAt(1)); // asigno a 'me' el elemento (1) (este es el vehiculo que esta en la posicion mas cerca de la interseccion)
            double x = me.position();// obtengo mi posicion
            //System.out.println(" in intersecctionCheck FromTabay: FT posicion del vehiculo mas cercano a la inter: " + x);

            double gapFT = laneLength - x - me.length();
            if (gapFT >= 0 && gapFT <= 5) {
                if (nvehToValle >= 1) {
                    lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                    double posTV = lastToValle.position();
                    if (posTV >= spaceMin) {
                        if (nvehToMerida >= 1) {
                            lastToMerida = (Moveable) (simcanvas.toMerida.street.elementAt(nvehToMerida - 1));
                            //      System.out.println(" in intersecctionCheck FromTabay: 1 Cambio de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                        } else {
                            //    System.out.println(" in intersecctionCheck FromTabay: 2 Cambio de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                        }
                    }
                } else {
                    if (nvehToMerida >= 1) {
                        lastToMerida = (Moveable) (simcanvas.toMerida.street.elementAt(nvehToMerida - 1));
                        //      System.out.println(" in intersecctionCheck FromTabay: 1 Cambio de Tabay a Merida");
                        changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                    } else {
                        //    System.out.println(" in intersecctionCheck FromTabay: 2 Cambio de Tabay a Merida");
                        changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                    }
                }
            }
        }//fin si solo veh desde Tabay


        // Vehiculo(s) en el canal desde Tabay y en el canal desde Merida 
        if (nvehFromTabay >= 1 && nvehFromMerida >= 1) {
            //System.out.println(" in intersecctionCheck FromTabay: (nvehFromTabay >= 1 && nvehFromMerida >= 1)");
            // primer vehiculo desde Tabay (mas cercano a la interseccion)
            me = (Moveable) (street.elementAt(1)); // 
            double x = me.position();// obtengo mi posicion
            //System.out.println(" in intersecctionCheck FromTabay: posicion del vehiculo mas cercano a la inter: " + x);

            // primer vehiculo desde Merida (mas cercano a la interseccion)
            firstFromMerida = (Moveable) (simcanvas.fromMerida.street.elementAt(1));
            double posFM = firstFromMerida.position();

            //System.out.println(" in intersecctionCheck FromTabay: FM posFM " + posFM);

            //double gapFT = laneLength - x - me.length() - ((Obstacle) street.elementAt(0)).length();
            double gapFT = laneLength - x - me.length();
            double gapFM = simcanvas.fromMerida.length() - posFM - firstFromMerida.length();
            //System.out.println(" in intersectionCheck FromTabay: brecha FM " + gapFM + " brecha FT" + gapFT);
            if ((gapFM >= 0 && gapFM <= 5) && (gapFT >= 0 && gapFT <= 5)) { // si los vehiculos estan en la interseccion

                //  System.out.println(" in intersecctionCheck FromTabay: ((gapFM >= 0 && gapFM <= 1) && (gapFT >= 0 && gapFT <= 1)) ");
                //System.out.println(" in intersecctionCheck FromTabay: gapFM " + gapFM + " gapFT " + gapFT);
                if ((me.priority() == 1 && firstFromMerida.priority() == 0)) { // el veh desde merida Va hacia el valle
                    //  System.out.println(" in intersecctionCheck FromTabay: me.destino()= " + firstFromMerida.destino() + " && me.priority()= " + me.priority());
                    if (nvehToValle >= 1) {
                        lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                        double posTV = lastToValle.position();
                        if (posTV >= spaceMin) {
                            if (nvehToMerida >= 1) {
                                lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                                //            System.out.println(" in intersecctionCheck FromTabay: 3 Cambiar de Tabay hacia Merida");
                                changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);// me=from, lastToValle=to                 
                            } else {
                                //          System.out.println(" in intersecctionCheck FromTabay: 4 Cambio de Tabay a Merida");
                                changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);// me=from, lastToValle=to                 
                            }
                        }
                    } else {
                        if (nvehToMerida >= 1) {
                            lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                            //            System.out.println(" in intersecctionCheck FromTabay: 3 Cambiar de Tabay hacia Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);// me=from, lastToValle=to                 
                        } else {
                            //          System.out.println(" in intersecctionCheck FromTabay: 4 Cambio de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);// me=from, lastToValle=to                 
                        }
                    }
                }
                if ((me.priority() == 0 && firstFromMerida.priority() == 0)){
                    System.out.println(" in intersecctionCheck FromTabay2: Prioridades iguales " + me.priority() + " && " + firstFromMerida.priority());
                    //me.setPriority(0);
                    firstFromMerida.setPriority(1);
                    System.out.println(" in intersecctionCheck FromTabay2: Nuevas Prioridades  " + me.priority() + " && " + firstFromMerida.priority());

                }
                if ((me.priority() == 1 && firstFromMerida.priority() == 1)){
                    System.out.println(" in intersecctionCheck FromTabay2: Prioridades iguales " + me.priority() + " && " + firstFromMerida.priority());
                    //me.setPriority(1);
                    firstFromMerida.setPriority(0);
                    System.out.println(" in intersecctionCheck FromTabay2: Nuevas Prioridades  " + me.priority() + " && " + firstFromMerida.priority());

                }
            } else {
                if ((gapFM > 5) && (gapFT >= 0 && gapFT <= 5)) {
                    //if (me.priority() == 0 || me.priority() == 1) {
                    //System.out.println(" in intersecctionCheck FromTabay: me.destino()= " + firstFromMerida.destino() + " && me.priority()= " + me.priority());
                    //System.out.println(" in intersecctionCheck FromTabay: ((gapFM > 1) && (gapFT>=0 && gapFT <= 1))");
                    if (nvehToValle >= 1) {
                        lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                        double posTV = lastToValle.position();
                        if (posTV >= spaceMin) {
                            if (nvehToMerida >= 1) {
                                lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                                //          System.out.println(" in intersecctionCheck FromTabay: 5 Cambiar de Tabay a Merida");
                                changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                            } else {
                                //        System.out.println(" in intersecctionCheck FromTabay: 6 Cambiar de Tabay a Merida");
                                changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                            }
                        }
                    } else if (nvehToValle < 1) {
                        if (nvehToMerida >= 1) {
                            lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);

                            //  System.out.println(" in intersecctionCheck FromTabay: 7 Cambiar de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                        } else {
                            //System.out.println(" in intersecctionCheck FromTabay: 8 Cambiar de Tabay a Merida");
                            changeFromTo(nvehToMerida, nvehFromTabay, spaceMin, me, lastToMerida, this, simcanvas.toMerida);
                        }
                    }
                    //}
                }
            }

        }// else, no hay vehiculos en ambos canales
    }

    private void changeFromTo(int nvehTo, int nvehFrom, double spaceMin, Moveable from, Moveable to, MicroStreet streetFrom, MicroStreet streetTo) {
        double vel = 11.; // para la velocidad de To(Valle, Tabay and Merida)
        if (nvehTo == 0) {
            double atime = from.arriveTime();
            from.setArriveTime(atime);
            from.setPosition(0);
            from.setLaneChange(this.laneChangeModel);

            // velocidad
            MicroModel model = from.model();
            double vNew;
            if (model == idmCarFT) {
                vNew = idmCarFT.Veq(vel);// asigna la velocidad
                System.out.println(" Velocidad FromTabay to Merida " + vNew);
            } else {
                vNew = idmTruckFT.Veq(vel);
            }
            from.setVelocity(vNew);

            streetTo.street.insertElementAt(from, nvehTo);
            streetFrom.street.removeElementAt(1);
            nvehTo++;
            nvehFrom--;
            return;
        }

        if (nvehTo >= 1) {
            to = (Moveable) (streetTo.street.elementAt(nvehTo - 1));
            double posFront = to.position();// posicion

            if (posFront > spaceMin - 10) {// si esa posicion es mayor que el espacio minimo entonces se inserta
                double atime = from.arriveTime();
                from.setArriveTime(atime);;
                from.setPosition(0);
                //from.setColor(Color.white);
                from.setLaneChange(this.laneChangeModel);

                // velocidad
                MicroModel model = from.model();
                double vNew;
                if (model == idmCarFT) {
                    vNew = idmCarFT.Veq(vel);// asigna la velocidad
                } else {
                    vNew = idmTruckFT.Veq(vel);
                }
                from.setVelocity(vNew);

                streetTo.street.insertElementAt(from, nvehTo);
                streetFrom.street.removeElementAt(1);
                nvehTo++;
                nvehFrom--;
                return;
            }
        }
    }
}
