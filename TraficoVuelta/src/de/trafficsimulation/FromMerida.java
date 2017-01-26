/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

import java.awt.Color;
import java.util.Date;

/**
 *
 * @author eimar
 */
public class FromMerida extends MicroStreet {

    private double laneLength;// tamaño del canal desde la redoma hasta la interseccion
    private ToValle toValle;// para actualizar los vectores de la via hacia el Valle
    private ToTabay toTabay;// para actualizar los vectores de la via hacia Tabay    
    private ToMerida toMerida;
    private FromValle fromValle;
    private FromTabay fromTabay;
    private Moveable me;// yo en el canal desde Merida
    private Moveable lastToTabay; // ultimo vehiculo en el canal hacia Tabay
    private Moveable firstFromTabay;// primer vehiculo en desde Tabay
    private Moveable lastToMerida;
    private Moveable lastToValle;
    private SimCanvas simcanvas;

    //Modelo IDM al inicio del canal
    //protected MicroModel idmCarFT = new IDMCarFM(); 
    //protected MicroModel idmTruckFT = new IDMTruckFM();
    // para las curvas subiendo
    //protected MicroModel syncCarFT = new IDMSyncCarFM();
    //protected MicroModel syncTruckFT = new IDMSyncTruckFM();
    public FromMerida(SimCanvas simcanvas, double laneLength, double p_factor, double deltaB, int choice_Szen, int prior) {

        super(laneLength, 0., p_factor, deltaB, 0, choice_Szen);


        this.toTabay = simcanvas.toTabay;
        this.toValle = simcanvas.toValle;
        this.toMerida = simcanvas.toMerida;
        this.fromValle = simcanvas.fromValle;
        this.fromTabay = simcanvas.fromTabay;
        this.simcanvas = simcanvas;

        this.laneLength = laneLength;


        street.addElement(new Obstacle(roadLength, 0, 0., 0));
        //street.addElement(new Car(0.0, idmCar.Veq(roadLength), 0, idmCar, inconsiderate,
        //      PKW_LENGTH_M, colorCarFM, 2));

    }

// Actualiza el canal (desdeMerida)
    public void updateFromMerida(double time, double dt, double qIn, double qIn1, double perTr, int prior) {

        this.time = time;

        // para actualizar las viejas posiciones
        //System.out.println("--------------FromMerida: old pos---------------");
        old_pos = setPos();

        accelerate(dt); // calcula la nueva velocidad
        translate(dt);// translada el vehiculo una posicion adelante

        //System.out.println("--------------FromMerida: new pos---------------");
        positions = setPos();

        //System.out.println("--------------FromMerida: tiempo de llegada--------------");
        //timeServ = setTimes();
        velocities = setVel();
        colors = setColors();
        lengths = setLengths();

        //inFlowFromMerida(this.time, dt, qIn, perTr);
        if (choice_Szen == 7 || choice_Szen == 8) {
            inFlowFromMerida1(this.time, dt, qIn, perTr, prior);
            intersecctionCheck1();
        }
        if (choice_Szen == 9 || choice_Szen == 10) {
            inFlowFromMerida2(this.time, dt, qIn1, perTr, prior);
            intersecctionCheck2();
        }


    }

    // para el calculo de la nueva velocidad
    protected void accelerate(double dt) {

        int imax = street.size() - 1;
        for (int i = imax; i >= 1; i--) {
            Moveable me = (Moveable) street.elementAt(i);
            Moveable vehTo = (Moveable) street.elementAt(i - 1);

            //Llamada al metodo accelerate de la interfaz Moveable implementado en la clase Car
            me.accelerate(dt, vehTo);
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

        }
        return street.size();
    }

    private void inFlowFromMerida1(double time, double dt, double qIn, double perTr, int prior) {
        System.out.println(" En inFlowFromMerida: laneLength= " + laneLength);
        final double spaceMin = 10;// tamaño de un camion + la brecha
        double space = 0;
        //int contPrior=0; //contador de prioridade de pasar (igual a 1)

        nin = nin + qIn * dt;// = 0+0,1*0,25= 0,111
        //System.out.println(" En inFlowFromMerida: nin = " + nin);
        if (nin > 1.0) {  // new vehicle imposed by the inflow BC, si num veh in>1 
            nin -= 1.0; // le resto 1 a nin
            int iPrev = street.size() - 1;// indice del veh previo, ejemp: 5-1=4
            if (iPrev >= 0) { // si 4>0
                //System.out.println(" En inFlowFromMerida: iPrev >= 0, vector con elementos ");
                space = ((Moveable) (street.elementAt(iPrev))).position();// posicion del vehiculo de enfrente
                //System.out.println(" En inFlowFromMerida: iprev = " + iPrev + ", posicion = " + space + ", spaceMin = " + spaceMin);
            } // si no
            else {   //esta vacio             
                //System.out.println(" En inFlowFromMerida: iPrev < 0, vector vacio ");
                space = spaceMin + 0.11; // la posicion de ese vehiculo es mayor que el espacio min
                //System.out.println(" En inFlowFromMerida: iprev = " + iPrev + ", posicion = " + space + ", spaceMin = " + spaceMin);
            }


            // ¿El espacio es suficiente para el nuevo vehiculo que ingresa?
            if (!(red = (space < spaceMin))) {// !(bandera = (20<5)), entonces, se inserta

                double rand = random.nextDouble() * 1.0;// numero aleatorio (double) entre 0y1
                
                //double rand = random.setSeed(new Date().getTime());// genera numeros aleatorios distintos en cada corrida
                
                //int randInt = Math.abs(random.nextInt());// numero aleatorio (int) positivo
                //MicroModel modelNew = (rand < perTr) ? idmTruck : idmCar; // Micromodel para la velocidad y la aceleracion
                MicroModel modelNew;// = (rand < perTr) ? idmTruckFM : idmCarFM;
                double vNew;// = modelNew.Veq(space);// asigna la velocidad
                double lNew;// = (rand < perTr) ? LKW_LENGTH_M : PKW_LENGTH_M;
                // asigna el tamaño del vehiculo
                //double lNew = (rand < perTr) ? PKW_LENGTH_M : LKW_LENGTH_M;
                //Color colorNew = (rand < perTr) ? colorTruck : colorCarFM; // asigna el color, rojo o negro
                int origin = ORIGINFM;
                double arriveTime = time;


                if (choice_Szen == 7 || choice_Szen == 8) {
                    int destino = (rand < PERCAR_FM1) ? 0 : 1;// 0 hacia el valle, 1 hacia Tabay  

                    if (destino == 0) {
                        double rand2 = random.nextDouble() * 1.0;
                        lNew = (rand2 < PERTRUCK_FMTV) ? LKW_LENGTH_M : PKW_LENGTH_M;
                        modelNew = (rand2 < PERTRUCK_FMTV) ? idmTruckFM : idmCarFM;
                        vNew = modelNew.Veq(space);// asigna la velocidad
                        int prioridad;
                        if (contPrior == prior) {
                            System.out.println(" En inFlowFromMerida1: volver a cero el contador de prioridades, contPrior = " + contPrior);
                            prioridad = 0;
                            contPrior = 0;
                        } else {
                            //System.out.println(" En inFlowFromMerida1: contPrior = " + contPrior);
                            prioridad = 1;
                            //System.out.println(" En inFlowFromMerida1: Prioridad = " + prioridad);
                            contPrior++;
                        }
                        //se creo un nuevo constructor por defecto para pasarle el origen, destino y prioridad del vehiculo
                        street.insertElementAt((new Car(0.0, vNew, origin, destino, prioridad, modelNew, arriveTime,
                                lNew)), street.size());

                    } else {
                        double rand2 = random.nextDouble() * 1.0;
                        lNew = (rand2 < PERTRUCK_FMTT) ? LKW_LENGTH_M : PKW_LENGTH_M;
                        modelNew = (rand2 < PERTRUCK_FMTT) ? idmTruckFM : idmCarFM;
                        vNew = modelNew.Veq(space);// asigna la velocidad
                        //se creo un nuevo constructor por defecto sin prioridad
                        street.insertElementAt((new Car(0.0, vNew, origin, destino, modelNew, arriveTime,
                                lNew)), street.size());
                    }

                }

            }
        }

    }

    private void inFlowFromMerida2(double time, double dt, double qIn, double perTr, int prior) {
        System.out.println(" En inFlowFromMerida: laneLength= " + laneLength);
        final double spaceMin = 10;// tamaño de un camion + la brecha
        double space = 0;
        //int contPrior=0; //contador de prioridade de pasar (igual a 1)

        nin = nin + qIn * dt;// = 0+0,1*0,25= 0,111
        //System.out.println(" En inFlowFromMerida: nin = " + nin);
        if (nin > 1.0) {  // new vehicle imposed by the inflow BC, si num veh in>1 
            nin -= 1.0; // le resto 1 a nin
            int iPrev = street.size() - 1;// indice del veh previo, ejemp: 5-1=4
            if (iPrev >= 0) { // si 4>0
                //System.out.println(" En inFlowFromMerida: iPrev >= 0, vector con elementos ");
                space = ((Moveable) (street.elementAt(iPrev))).position();// posicion del vehiculo de enfrente
                //System.out.println(" En inFlowFromMerida: iprev = " + iPrev + ", posicion = " + space + ", spaceMin = " + spaceMin);
            } // si no
            else {   //esta vacio             
                //System.out.println(" En inFlowFromMerida: iPrev < 0, vector vacio ");
                space = spaceMin + 0.11; // la posicion de ese vehiculo es mayor que el espacio min
                //System.out.println(" En inFlowFromMerida: iprev = " + iPrev + ", posicion = " + space + ", spaceMin = " + spaceMin);
            }


            // ¿El espacio es suficiente para el nuevo vehiculo que ingresa?
            if (!(red = (space < spaceMin))) {// !(bandera = (20<5)), entonces, se inserta

                double rand = random.nextDouble() * 1.0;// numero aleatorio (double) entre 0y1
                //int randInt = Math.abs(random.nextInt());// numero aleatorio (int) positivo
                //MicroModel modelNew = (rand < perTr) ? idmTruck : idmCar; // Micromodel para la velocidad y la aceleracion
                MicroModel modelNew = (rand < PERTRUCK_FMTV) ? idmTruckFM : idmCarFM;
                double vNew = modelNew.Veq(space);// asigna la velocidad
                double lNew = (rand < PERTRUCK_FMTV) ? LKW_LENGTH_M : PKW_LENGTH_M; // asigna el tamaño del vehiculo
                //double lNew = (rand < perTr) ? PKW_LENGTH_M : LKW_LENGTH_M;
                //Color colorNew = (rand < perTr) ? colorTruck : colorCarFM; // asigna el color, rojo o negro
                int origin = ORIGINFM;
                double arriveTime = time;


                if (choice_Szen == 9 || choice_Szen == 10) {
                    //Ampliacion From Merida y Ambos canales
                    //prioridad de pasar solo para old From Merida y no para NewFromMerida
                    int destino = 0;
                    //int prioridad = (rand < PRIORFM) ? 0 : 1;
                    int prioridad;
                    if (contPrior == prior) {
                        System.out.println(" En inFlowFromMerida1: volver a cero el contador de prioridades, contPrior = " + contPrior);
                        prioridad = 0;
                        contPrior = 0;
                    } else {
                        System.out.println(" En inFlowFromMerida1: contPrior = " + contPrior);
                        prioridad = 1;
                        System.out.println(" En inFlowFromMerida1: Prioridad = " + prioridad);
                        contPrior++;
                    }
                    //se creo un nuevo constructor por defecto para pasarle el origen, destino y prioridad del vehiculo
                    street.insertElementAt((new Car(0.0, vNew, origin, destino, prioridad, modelNew, arriveTime,
                            lNew)), street.size());

                }
            }
        }

    }

    private void intersecctionCheck1() {
        final double spaceMin = CRUCE / 2; // tamaño de un camion = 8., mas brecha minima = 2
        //double gap = 10.;

        int nvehFromMerida = street.size() - 1; // nuemro de vehiculos en el canal desde Merida
        //System.out.println("in intersecctionCheck FromMerida: desde Merida= " + nvehFromMerida);

        int nvehFromTabay = simcanvas.fromTabay.street.size() - 1;// numero de vehiculos en el canal desde Tabay
        //System.out.println("in intersecctionCheck FromMerida:  desde Tabay " + nvehFromTabay);


        int nvehToValle = simcanvas.toValle.street.size();// numero de vehiculos en el canal hacia el Valle
        //System.out.println("in intersecctionCheck FromMerida :  hacia el Valle= " + nvehToValle);

        int nvehToTabay = simcanvas.toTabay.street.size() - 1;// numero de vehiculos en el canal hacia Tabay
        //System.out.println("in intersecctionCheckFromMerida:  hacia Tabay= " + nvehToTabay);

        int nvehToMerida = simcanvas.toMerida.street.size();// numero de vehiculos en el canal hacia Merida
        //System.out.println("in intersecctionCheck FromMerida:  hacia Merida= " + nvehToMerida);

        //System.out.println(" in intersectionCheck FromMerida: Tamaño del canal de Tabay " + simcanvas.fromTabay.roadLength);
        //System.out.println(" in intersectionCheck FromMerida: Tamaño del canal de Merida " + roadLength);
        //System.out.println(" in intersectionCheck FromMerida: Tamaño del canal de El Valle " + simcanvas.fromValle.roadLength);


        if (nvehFromTabay < 1 && nvehFromMerida >= 1) {
            //System.out.println(" in intersecctionCheck FromMerida: (nvehFromTabay < 1 && nvehFromMerida >= 1)");
            me = (Moveable) (street.elementAt(1));
            double x = me.position();
            double gapFM = laneLength - x - me.length();
            if ((gapFM >= 0 && gapFM <= 5)) {
                if (me.destino() == 0) {
                    //System.out.println(" in intersecctionCheck FromMerida1: Hacia El Valle me.destino()= " + me.destino());
                    if (nvehToMerida >= 1) {
                        lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                        double posTM = lastToMerida.position();
                        if (posTM >= spaceMin) {
                            if (nvehToValle >= 1) {
                                lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                                //            System.out.println(" in intersecctionCheck FromMerida: 5 Cambiar de Merida hacia el valle");
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                            } else {
                                //          System.out.println(" in intersecctionCheck FromMerida: 6 Cambiar de Merida a El Valle");
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                            }
                        }
                    } else {
                        if (nvehToValle >= 1) {
                            lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                            //System.out.println(" in intersecctionCheck FromMerida: 1 Cambiar de Merida a El Valle");
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// firstFromMerida=from, lastToValle=to
                        } else {
                            //System.out.println(" in intersecctionCheck FromMerida: 2 Cambiar de Merida a El Valle");
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);
                        }
                    }


                } else {
                    //System.out.println(" in intersecctionCheck FromMerida1: Hacia Tabay me.destino()= " + me.destino());
                    if (nvehToTabay >= 1) {
                        lastToTabay = (Moveable) simcanvas.toTabay.street.elementAt(nvehToTabay - 1);
                        //System.out.println(" in intersecctionCheck FromMerida: 3 Cambiar de Merida a Tabay");
                        changeFromTo(nvehToTabay, nvehFromMerida, me, lastToTabay, this, simcanvas.toTabay);
                    } else {
                        //System.out.println(" in intersecctionCheck FromMerida: 4 Cambiar de Merida a El Valle");
                        changeFromTo(nvehToTabay, nvehFromMerida, me, lastToTabay, this, simcanvas.toTabay);
                    }

                }
            }
        }// fin si solo veh desde Merida

        // Vehiculo(s) en canal desde Tabay y en el canal desde Merida 
        if (nvehFromTabay >= 1 && nvehFromMerida >= 1) {
            System.out.println(" in intersecctionCheck FromMerida1: (nvehFromTabay >= 1 && nvehFromMerida >= 1)");
            // primer vehiculo desde Merida (mas cercano a la interseccion)
            me = (Moveable) (street.elementAt(1)); // 
            double x = me.position();// obtengo mi posicion
            System.out.println(" in intersecctionCheck FromMerida1: mi posicion : " + x);

            // primer vehiculo desde Tabay (mas cercano a la interseccion)

            firstFromTabay = (Moveable) (simcanvas.fromTabay.street.elementAt(1));
            double posFT = firstFromTabay.position();
            System.out.println(" in intersecctionCheck FromMerida1: posicion del veh de FromTabay mas cercano " + posFT + " S0 " + S0_INIT_M);

            double gapFT = simcanvas.fromTabay.length() - posFT - firstFromTabay.length();
            //System.out.println(" in intersectionCheck FromMerida: tamaño fromTabay= "+simcanvas.fromTabay.length()+" posFT "+ posFT+" firstFromTabay.length()= "+firstFromTabay.length()+" tamaño del obstaculo= "+((Moveable) (simcanvas.fromTabay.street.elementAt(0))).length());

            double gapFM = laneLength - x - me.length();//=tamaño del canal-mi pos-mi tam
            //System.out.println(" in intersectionCheck FromMerida1: brecha FM " + gapFM + " brecha FT" + gapFT);


            if (((gapFM >= 0 && gapFM <= 5) && (gapFT >= 0 && gapFT <= 5))) { // si los vehiculos estan en la interseccion

                //System.out.println(" in intersecctionCheck FromMerida1: ((gapFM >= 0 && gapFM <= 2) && (gapFT >= 0 && gapFT <= 2)) ");
                //System.out.println(" in intersecctionCheck FromMerida: gapFM " + gapFM + " gapFT" + gapFT);
                //if (me.destino() == 0 && firstFromTabay.priority() == 0) {// el veh desde merida Va hacia el valle
                if (me.destino() == 0) {
                    //System.out.println(" in intersecctionCheck FromMerida1: Hacia El Valle me.destino()= " + me.destino());
                    if ((me.priority() == 1) && firstFromTabay.priority() == 0) {
                        //System.out.println(" in intersecctionCheck FromMerida1: me.priority= " + me.priority() + " && firstFromTabay.priority()= " + firstFromTabay.priority());
                        if (nvehToMerida >= 1) {
                            lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                            double posTM = lastToMerida.position();
                            if (posTM >= spaceMin) {
                                if (nvehToValle >= 1) {
                                    lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                                    //            System.out.println(" in intersecctionCheck FromMerida: 5 Cambiar de Merida hacia el valle");
                                    changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                                } else {
                                    //          System.out.println(" in intersecctionCheck FromMerida: 6 Cambiar de Merida a El Valle");
                                    changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                                }
                            }
                        } else {
                            if (nvehToValle >= 1) {
                                lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                                //    System.out.println(" in intersecctionCheck FromMerida: 7 Cambiar de Merida hacia el valle");
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                            } else {
                                //  System.out.println(" in intersecctionCheck FromMerida: 8 Cambiar de Merida a El Valle");
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                            }

                        }
                    }

                } else if (me.destino() == 1) {
                    //System.out.println(" in intersecctionCheck FromMerida1: Hacia Tabay me.destino()= " + me.destino());
                    if (nvehToTabay >= 1) {
                        lastToTabay = (Moveable) simcanvas.toTabay.street.elementAt(nvehToTabay - 1);
                        //System.out.println(" in intersecctionCheck FromMerida: Cambiar de Merida a Tabay");
                        changeFromTo(nvehToTabay, nvehFromMerida, me, lastToTabay, this, simcanvas.toTabay);
                    } else {
                        changeFromTo(nvehToTabay, nvehFromMerida, me, lastToTabay, this, simcanvas.toTabay);
                    }
                }

            } else if ((gapFM >= 0 && gapFM <= 5) && (gapFT > 5)) {

                //System.out.println(" in intersecctionCheck FromMerida1: ((gapFM >= 0 && gapFM <= 2) && (gapFT >= 2)) ");
                //System.out.println(" in intersecctionCheck FromMerida1: gapFM " + gapFM + " gapFT " + gapFT);
                if (me.destino() == 0) {
                    //System.out.println(" in intersecctionCheck FromMerida1: Hacia el valle, me.destino()= " + me.destino());
                    if (nvehToMerida >= 1) {

                        lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                        double posTM = lastToMerida.position();
                        //System.out.println(" in intersecctionCheck FromMerida1: Si hay vehiculos hacia merida en la pos: " + posTM);
                        if (posTM >= spaceMin) {
                            //System.out.println(" in intersecctionCheck FromMerida1: no hay veh en la inter hacia merida ¿Habra hacia el Valle? ");
                            if (nvehToValle >= 1) {
                                //System.out.println(" in intersecctionCheck FromMerida1: tambien hay veh hacia el Valle");
                                lastToValle = (Moveable) (simcanvas.toValle.street.elementAt(nvehToValle - 1));
                                //System.out.println(" in intersecctionCheck FromMerida1: Cambiar de Merida al Valle");
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// firstFromMerida=from, lastToValle=to
                            } else {
                                //System.out.println(" in intersecctionCheck FromMerida1: si se puede hacia el Valle. Cambiar");
                                //System.out.println(" in intersecctionCheck FromMerida1: no hay veh hacia el Valle" );
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);
                            }
                        }
                    } else {
                        //System.out.println(" in intersecctionCheck FromMerida1: no hay veh hacia Merida");
                        if (nvehToValle >= 1) {
                            //System.out.println(" in intersecctionCheck FromMerida1: hay veh hacia el valle");
                            lastToValle = (Moveable) (simcanvas.toValle.street.elementAt(nvehToValle - 1));
                            if (lastToValle.position() > me.length()) {
                                //System.out.println(" in intersecctionCheck FromMerida1: Cambiar de Merida al Valle");
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// firstFromMerida=from, lastToValle=to
                            } else {
                                //System.out.println(" in intersecctionCheck FromMerida1: hay veh hacia el valle");
                            }
                        } else {
                            //System.out.println(" in intersecctionCheck FromMerida1: no hay veh hacia el valle");
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);
                        }
                    }
                } else {
                    //System.out.println(" in intersecctionCheck FromMerida1: Hacia Tabay, me.destino()= " + me.destino());
                    if (nvehToTabay >= 1) {
                        lastToTabay = (Moveable) simcanvas.toTabay.street.elementAt(nvehToTabay - 1);
                        //    System.out.println(" in intersecctionCheck FromMerida: Cambiar de Merida a Tabay");
                        changeFromTo(nvehToTabay, nvehFromMerida, me, lastToTabay, this, simcanvas.toTabay);
                    } else {
                        changeFromTo(nvehToTabay, nvehFromMerida, me, lastToTabay, this, simcanvas.toTabay);
                    }
                }
            }



        }// else, no hay vehiculos en ambos canales
    }

    private void intersecctionCheck2() {
        final double spaceMin = SPACEMIN / 2; // tamaño de un camion = 8., mas brecha minima = 2
        //double gap = 10.;

        int nvehFromMerida = simcanvas.fromMerida.street.size() - 1; // nuemro de vehiculos en el canal desde Merida
        //System.out.println("in intersecctionCheck FromMerida: desde Merida= " + nvehFromMerida);

        int nvehFromTabay = simcanvas.fromTabay.street.size() - 1;// numero de vehiculos en el canal desde Tabay
        //System.out.println("in intersecctionCheck FromMerida:  desde Tabay " + nvehFromTabay);

        int nvehToValle = simcanvas.toValle.street.size();// numero de vehiculos en el canal hacia el Valle
        //System.out.println("in intersecctionCheck FromMerida :  hacia el Valle= " + nvehToValle);

        int nvehToMerida = simcanvas.toMerida.street.size();// numero de vehiculos en el canal hacia Merida
        //System.out.println("in intersecctionCheck FromMerida:  hacia Merida= " + nvehToMerida);

        if (nvehFromTabay < 1 && nvehFromMerida >= 1) {
            //System.out.println(" in intersecctionCheck FromMerida2: (nvehFromTabay < 1 && nvehFromMerida >= 1)");
            me = (Moveable) (street.elementAt(1));
            double x = me.position();
            double gapFM = laneLength - x - me.length();
            if ((gapFM >= 0 && gapFM <= 5)) {
                //if (me.destino() == 0) {
                //System.out.println(" in intersecctionCheck FromMerida1: Hacia El Valle me.destino()= " + me.destino());
                //System.out.println(" in intersecctionCheck FromMerida2: (gapFM >= 0 && gapFM <= 2)");
                if (nvehToMerida >= 1) {
                    lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                    double posTM = lastToMerida.position();
                    if (posTM >= spaceMin) {
                        if (nvehToValle >= 1) {
                            lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                            //            System.out.println(" in intersecctionCheck FromMerida: 5 Cambiar de Merida hacia el valle");
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                        } else {
                            //          System.out.println(" in intersecctionCheck FromMerida: 6 Cambiar de Merida a El Valle");
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                        }
                    }
                } else {
                    if (nvehToValle >= 1) {
                        lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                        //System.out.println(" in intersecctionCheck FromMerida: 1 Cambiar de Merida a El Valle");
                        changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// firstFromMerida=from, lastToValle=to
                    } else {
                        //System.out.println(" in intersecctionCheck FromMerida: 2 Cambiar de Merida a El Valle");
                        changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);
                    }
                }
                // }
            }

        }// fin si solo veh desde Merida

        // Vehiculo(s) en canal desde Tabay y en el canal desde Merida 
        if (nvehFromTabay >= 1 && nvehFromMerida >= 1) {
            //System.out.println(" in intersecctionCheck FromMerida2: (nvehFromTabay >= 1 && nvehFromMerida >= 1)");
            // primer vehiculo desde Merida (mas cercano a la interseccion)
            me = (Moveable) (simcanvas.fromMerida.street.elementAt(1)); // 
            double x = me.position();// obtengo mi posicion
            //System.out.println(" in intersecctionCheck FromMerida: mi posicion : " + x);

            // primer vehiculo desde Tabay (mas cercano a la interseccion)
            firstFromTabay = (Moveable) (simcanvas.fromTabay.street.elementAt(1));
            double posFT = firstFromTabay.position();
            //System.out.println(" in intersecctionCheck FromMerida: posicion del veh de FromTabay mas cercano " + posFT + " S0 " + S0_INIT_M);

            double gapFT = simcanvas.fromTabay.length() - posFT - firstFromTabay.length();

            double gapFM = laneLength - x - me.length();//=tamaño del canal-mi pos-mi tam
            //System.out.println(" in intersectionCheck FromMerida: brecha FM " + gapFM + " brecha FT" + gapFT);


            if (((gapFM >= 0 && gapFM <= 5) && (gapFT >= 0 && gapFT <= 5))) { // si los vehiculos estan en la interseccion

                //System.out.println(" in intersecctionCheck FromMerida2: ((gapFM >= 0 && gapFM <= 2) && (gapFT >= 0 && gapFT <= 2)) ");
                //System.out.println(" in intersecctionCheck FromMerida: gapFM " + gapFM + " gapFT" + gapFT);
                if (me.priority() == 1 && firstFromTabay.priority() == 0) {// el veh desde merida Va hacia el valle

                    if (nvehToMerida >= 1) {
                        lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                        double posTM = lastToMerida.position();
                        if (posTM >= spaceMin) {
                            if (nvehToValle >= 1) {
                                lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                                //            System.out.println(" in intersecctionCheck FromMerida: 5 Cambiar de Merida hacia el valle");
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                            } else {
                                //          System.out.println(" in intersecctionCheck FromMerida: 6 Cambiar de Merida a El Valle");
                                changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                            }
                        }
                    } else {
                        if (nvehToValle >= 1) {
                            lastToValle = (Moveable) simcanvas.toValle.street.elementAt(nvehToValle - 1);
                            //    System.out.println(" in intersecctionCheck FromMerida: 7 Cambiar de Merida hacia el valle");
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                        } else {
                            //  System.out.println(" in intersecctionCheck FromMerida: 8 Cambiar de Merida a El Valle");
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// me=from, lastToValle=to                 
                        }

                    }
                }
            } else if ((gapFM >= 0 && gapFM <= 5) && (gapFT > 5)) {
                //System.out.println(" in intersecctionCheck FromMerida2: (gapFM >= 0 && gapFM <= 2) && (gapFT > 2)");
                if (nvehToMerida >= 1) {
                    lastToMerida = (Moveable) simcanvas.toMerida.street.elementAt(nvehToMerida - 1);
                    double posTM = lastToMerida.position();
                    if (posTM >= spaceMin) {

                        if (nvehToValle >= 1) {
                            lastToValle = (Moveable) (simcanvas.toValle.street.elementAt(nvehToValle - 1));
                            //              System.out.println(" in intersecctionCheck FromMerida: Cambiar de Merida al Valle");
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// firstFromMerida=from, lastToValle=to
                        } else {
                            changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);
                        }
                    }
                } else {
                    if (nvehToValle >= 1) {
                        lastToValle = (Moveable) (simcanvas.toValle.street.elementAt(nvehToValle - 1));
                        //              System.out.println(" in intersecctionCheck FromMerida: Cambiar de Merida al Valle");
                        changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);// firstFromMerida=from, lastToValle=to
                    } else {
                        changeFromTo(nvehToValle, nvehFromMerida, me, lastToValle, this, simcanvas.toValle);
                    }
                }

            }
            //}


        }// else, no hay vehiculos en ambos canales
    }

    public void changeFromTo(int nvehTo, int nvehFrom, Moveable from, Moveable to, MicroStreet streetFrom, MicroStreet streetTo) {
        //double vel = 11.; // para la velocidad de To(Valle, Tabay and Merida)
        if (nvehTo == 0) {
            //double atime = from.arriveTime();
            //from.setArriveTime(atime);
            //from.setLane(0);
            from.setPosition(0);

            // velocidad
            MicroModel model = from.model();
            //double vNew;
            if (from.destino() == 0) {
                if (model == idmCarFM) {
                    from.setModel(idmCarTV);
                    //vNew = idmCar.Veq(vel);// asigna la velocidad
                    System.out.println(" Carro");
                } else {
                    from.setModel(idmTruckTV);
                    //vNew = idmTruck.Veq(vel);
                    System.out.println(" Camion");
                }
            } else if (model == idmCarFM) {
                from.setModel(idmCarTT);
                //vNew = idmCar.Veq(vel);// asigna la velocidad
                System.out.println(" Carro");
            } else {
                from.setModel(idmTruckTT);
                //vNew = idmTruck.Veq(vel);
                System.out.println(" Camion");
            }

            //from.setVelocity(vNew);

            streetTo.street.insertElementAt(from, nvehTo);
            streetFrom.street.removeElementAt(1);
            nvehTo++;
            nvehFrom--;
            return;
        } else if (nvehTo >= 1) {
            to = (Moveable) (streetTo.street.elementAt(nvehTo - 1));
            double posFront = to.position();// posicion;

            if (posFront > from.length()) {// si esa posicion es mayor que el espacio minimo entonces se inserta
                //double atime = from.arriveTime();
                //from.setArriveTime(atime);
                from.setPosition(0);

                // velocidad
                MicroModel model = from.model();
                //double vNew;
                if (model == idmCarFM) {
                    //vNew = idmCar.Veq(vel);// asigna la velocidad
                    System.out.println(" Carro");
                } else {
                    //vNew = idmTruck.Veq(vel);
                    System.out.println(" Camion");
                }
                //from.setVelocity(vNew);

                streetTo.street.insertElementAt(from, nvehTo);
                streetFrom.street.removeElementAt(1);
                nvehTo++;
                nvehFrom--;
                return;
            }
        }

    }
}
