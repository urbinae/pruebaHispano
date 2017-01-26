/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

import java.awt.Color;

/**
 *
 * @author Eimar
 */
public class ToMerida extends MicroStreet {

    private double laneLength;// tamaño del canal desde la redoma hasta la interseccion
    Color color = colorCarTT;

    public ToMerida(double laneLength, double p_factor, double deltaB, int choice_Szen) {
        super(laneLength, 0., p_factor, deltaB, 0, choice_Szen);
        this.laneLength = laneLength;
        //street.addElement(new Obstacle(laneLength+200., 0, 1., 0));

    }

    public void updateToMerida(double time, double dt) {

        this.time = time;
        // para actualizar las viejas posiciones
        //System.out.println("--------------ToMerida: old pos---------------");
        old_pos = setPos();

        insertBCCars();

        accelerate(dt);
        clearBCCars();
        translate(dt);
        sort();



        //System.out.println("--------------ToMerida: new pos---------------");
        positions = setPos();
        velocities = setVel();
        colors = setColors();
        lengths = setLengths();

        System.out.println("-------------------------En ToMerida Tiempos--------------------------");
        //timesArrive();        
        outFlowTM(time);
        //calcTimesService();

    }

    private void outFlowTM(double time) {
        double eTime = time;
        int imax = street.size() - 1;
        //System.out.println(" en outFlow: "+ imax);
        if (imax >= 0) {
            //System.out.println("MicroStreet.oFlow: removing vehicle...");
            while ((imax >= 0) && // (imax>=0) first to prevent indexOutOf...!
                    (((Moveable) (street.elementAt(0))).position() >= STRAIGHT_TOMERIDA_M)) {
                //timeS = ((Moveable) (street.elementAt(0))).exitTime();//tiempo en el sistema
                //int i=exit.size();
                //exit.insertElementAt(new Double(eTime), i);// alamcena el tiempo de salida
                //Moveable exit = ((Moveable) (street.elementAt(0)));
                //exit.setExitTime(time);
                if (((Moveable) (street.elementAt(0))).origin() == 2) {// viene de Tabay
                    double aTime = ((Moveable) (street.elementAt(0))).arriveTime();
                    double sTime = eTime - aTime;//Tiempo de servicio
                    numVehFT++;
                    System.out.println(" Viene de Tabay -> Llegada " + aTime + " Salida " + eTime + " Servicio " + sTime);
                    sumTimesFT = sumTimesFT + sTime;
                    serviceTFT = sumTimesFT / numVehFT;// promedio tiempo de servicio
                    //serviceT = sTime;
                 }else
                if (((Moveable) (street.elementAt(0))).origin() == 3) {// viene de El Valle
                    double aTime = ((Moveable) (street.elementAt(0))).arriveTime();
                    double sTime = eTime - aTime;//Tiempo de servicio
                    numVehFV++;
                    System.out.println(" Viene de El Valle -> Llegada " + aTime + " Salida " + eTime + " Servicio " + sTime);
                    sumTimesFV = sumTimesFV + sTime;
                    serviceTFV = sumTimesFV / numVehFV;// promedio tiempo de servicio
                    //serviceT = sTime;
                }
                street.removeElementAt(0);
                imax--;
            }
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

            if ((x_old <= STRAIGHT_TOMERIDA_M - CRUCE) && (x_new > STRAIGHT_TOMERIDA_M - CRUCE)) {
                double rand = random.nextDouble() * 1.0;// numero aleatorio (double) entre 0y1
                if (rand < 0.5) {
                    if (me.origin() == 2) {
                        if (me.model() == syncTruckFT) {
                            me.setModel(syncTruckTM);
                        }else
                            me.setModel(syncCarTM);
                    } else 
                        if (me.model() == idmTruckFV) {
                            me.setModel(syncTruckTM);
                        }else
                            me.setModel(syncCarTM);
                    

                }
            }
        }
        return street.size();
    }
}
