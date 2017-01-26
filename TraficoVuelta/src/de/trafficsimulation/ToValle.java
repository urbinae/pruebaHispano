/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author eimar
 */
public class ToValle extends MicroStreet {

    private double laneLength;// tamaño del canal desde la redoma hasta la interseccion

    public ToValle(double laneLength, double p_factor, double deltaB, int choice_Szen) {
        
        super(laneLength, 0., p_factor, deltaB, 0, choice_Szen);
        
        this.laneLength=laneLength;
         //street.addElement(new Obstacle(20., 0, 1., 0));
    }

    public void updateToValle(double time, double dt) {

        this.time = time;
        
        // para actualizar las viejas posiciones
        //System.out.println("--------------ToValle: old pos---------------");
        old_pos = setPos();        // need old info for detectors and drawing
        
        insertBCCars();
        accelerate(dt);
        clearBCCars();
        translate(dt);
        sort();

        // no sink/source between determining positions and old pos etc!
        // -> no bookkeeping necessary for graph. output!

        //System.out.println("--------------ToValle: new pos---------------");
        positions = setPos();
        
        velocities = setVel();
        colors = setColors();
        lengths = setLengths();

        outFlow(time);

    }

    // para el calculo de la aceleracion
    protected void accelerate(double dt) {

        int imax = street.size() - 1;
        for (int i = imax; i >= 1; i--) {
            // Se obtiene mi auto y el del frente
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
        }
        return street.size();
    }
    
     public void outFlow(double time) {
        double eTime=time;
        int imax = street.size()-1;
        //System.out.println(" en outFlow: "+ imax);
        if (imax >= 0) {
             //System.out.println("MicroStreet.oFlow: removing vehicle...");
            while ((imax >= 0) && // (imax>=0) first to prevent indexOutOf...!
                    (((Moveable) (street.elementAt(0))).position() >= STRAIGHT_FROMTOVALLE_M)) {
                //timeS = ((Moveable) (street.elementAt(0))).exitTime();//tiempo en el sistema
                //int i=exit.size();
                //exit.insertElementAt(new Double(eTime), i);// alamcena el tiempo de salida
                //Moveable exit = ((Moveable) (street.elementAt(0)));
                //exit.setExitTime(time);
                double aTime = ((Moveable) (street.elementAt(0))).arriveTime();
                double sTime = eTime - aTime;//Tiempo de servicio
                numVehFMTV ++;
                System.out.println(" Llegada "+ aTime+" Salida "+ eTime+" Servicio "+sTime);
                sumTimesFMTV = sumTimesFMTV + sTime;
                serviceTFMTV = sumTimesFMTV/numVehFMTV;// promedio tiempo de servicio
                //serviceT = sTime;
                street.removeElementAt(0);
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
}
