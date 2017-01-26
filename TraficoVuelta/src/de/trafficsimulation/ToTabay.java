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
public class ToTabay extends MicroStreet {
    private double laneLength;// tamaño del canal desde la redoma hasta la interseccion
    Color color= colorCarTT;
       
    
    public ToTabay(double laneLength, double p_factor, double deltaB, int choice_Szen){
        super(laneLength, 0., p_factor, deltaB, 0, choice_Szen);
        this.laneLength=laneLength;
        
        upcurve1 = STRAIGHT1_M - 20;
        upcurve2 = (STRAIGHT1_M + Math.PI* RADIUS_M + STRAIGHT2_M) - 20.; 
        
        street.addElement(new Obstacle(roadLength+200., 0, 0., 0));
        
    }
    public void updateToTabay(double time, double dt) {

        this.time = time;

        // para actualizar las viejas posiciones
        //System.out.println("--------------ToTabay: old pos---------------");
        old_pos = setPos();       

        insertBCCars();

        accelerate(dt);
        clearBCCars();
        translate(dt);
        sort();


        //System.out.println("--------------ToTabay: new pos---------------");
        positions = setPos();
        velocities = setVel();
        colors = setColors();
        lengths = setLengths();
        
        System.out.println("-------------------------En ToTabay Tiempos--------------------------");
        //timesArrive();        
        outFlow(time);
        //calcTimesService();
       

    }

    // para el calculo de la aceleracion
    protected void accelerate(double dt) {

        int imax = street.size() - 1;
        for (int i = imax; i >= 1; i--) {
            Moveable me = (Moveable) street.elementAt(i);
            Moveable frontVeh = (Moveable) street.elementAt(i - 1);
            
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

            //if (choice_Szen == 7) {
                if ((x_old >= 0.) && (x_old < STRAIGHT1_M)){
                    //System.out.println("-------------------------En ToTabay Velocidades en la recta 1--------------------------");
                    
                    //System.out.println(" Velocidad:= "+me.velocity());
                }
                
                if ((x_old <= upcurve1) && (x_new > upcurve1)) {
                    
                //System.out.println("-------------------------En ToTabay Velocidades en la curva 1--------------------------");
                
                //System.out.println(" Velocidad: = "+me.velocity());
                    if (me.model() == idmTruckTT) {
                        me.setModel(syncTruckTT);
                    } else {
                        me.setModel(syncCarTT);
                    }
                }
                if((x_old >= STRAIGHT1_M + Math.PI*RADIUS_M) && (x_old < STRAIGHT1_M+STRAIGHT2_M+Math.PI*RADIUS_M) ){
                    //System.out.println("-------------------------En ToTabay Velocidades en la recta 2--------------------------");
                    
                    //System.out.println(" Velocidad:= "+me.velocity());
                }
                if ((x_old <= upcurve2) && (x_new > upcurve2)){
                    //System.out.println("-------------------------En ToTabay Velocidades en la curva 2--------------------------");
                     if (me.model() == syncTruckTT) {
                        me.setModel(syncTruckTT);
                    } else {
                        me.setModel(syncCarTT);
                    }
                    //System.out.println(" Velocidad:= "+me.velocity());
                }
                
                if ((x_old >= STRAIGHT1_M+STRAIGHT2_M+2*Math.PI*RADIUS_M) && (x_old < STRAIGHT1_M+STRAIGHT2_M+2*Math.PI*RADIUS_M+STRAIGHT3_M)){
                    //System.out.println("-------------------------En ToTabay Velocidades en la recta 3--------------------------");
                    
                    //System.out.println(" Velocidad:= "+me.velocity());
                }
                
                
            //}
        }
        return street.size();
    }
     
     public void outFlow(double time) {
        double eTime=time;
        int imax = street.size()-1;
        //System.out.println(" en outFlow: "+ imax);
        if (imax >= 1) {
             //System.out.println("MicroStreet.oFlow: removing vehicle...");
            while ((imax >= 1) && // (imax>=0) first to prevent indexOutOf...!
                    (((Moveable) (street.elementAt(1))).position() >= roadLength)) {
                //timeS = ((Moveable) (street.elementAt(0))).exitTime();//tiempo en el sistema
                //int i=exit.size();
                //exit.insertElementAt(new Double(eTime), i);// alamcena el tiempo de salida
                //Moveable exit = ((Moveable) (street.elementAt(0)));
                //exit.setExitTime(time);
                double aTime = ((Moveable) (street.elementAt(1))).arriveTime();
                double sTime = eTime - aTime;//Tiempo de servicio
                numVehFMTT ++;
                System.out.println(" Llegada "+ aTime+" Salida "+ eTime+" Servicio "+sTime);
                sumTimesFMTT = sumTimesFMTT + sTime;
                serviceTFMTT = sumTimesFMTT/numVehFMTT;// promedio tiempo de servicio
                //serviceT = sTime;
                street.removeElementAt(1);
                imax--;
            }
        }
    }
}
