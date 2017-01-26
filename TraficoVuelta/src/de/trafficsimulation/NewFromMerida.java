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
public class NewFromMerida extends MicroStreet {

    ToTabay toTabay;
    Moveable lastToTabay, me;
    SimCanvas simcanvas;
    double laneLength;

    public NewFromMerida(SimCanvas simcanvas, double laneLength, double mergingPos,
            double p_factor, double deltaB, int choice_Szen) {

        super(laneLength, 0., p_factor, deltaB, 0, choice_Szen);


        this.toTabay = simcanvas.toTabay;

        this.simcanvas = simcanvas;

        this.laneLength = laneLength;


        street.addElement(new Obstacle(roadLength, 0, 0., 0));
       

    }

    public void updateNewFromMerida(double time, double dt, double qIn, double perTr,
            double p_factor, double deltaB) {

        this.time = time;

        old_pos = setPos();

        accelerate(dt); // calcula la nueva velocidad
        translate(dt);// translada el vehiculo una posicion adelante

        positions = setPos();

        velocities = setVel();
        colors = setColors();
        lengths = setLengths();


        inFlowFromMerida2(this.time, dt, qIn, perTr);
        changeToTabay();
    }
    
    private void inFlowFromMerida2(double time, double dt, double qIn, double perTr) {
        System.out.println(" En inFlowFromMerida: laneLength= " + laneLength);
        final double spaceMin = 10;
        double space = 0;

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
                MicroModel modelNew = (rand < PERTRUCK_FMTT) ? idmTruckFM : idmCarFM;
                double vNew = modelNew.Veq(space);// asigna la velocidad
                double lNew = (rand < PERTRUCK_FMTT) ? LKW_LENGTH_M : PKW_LENGTH_M; // asigna el tamaño del vehiculo
                //Color colorNew = (rand < perTr) ? colorTruck : colorCarFM; // asigna el color, rojo o negro
                int origin=ORIGINFM;
                
                int destino = 1; 
                street.insertElementAt((new Car(0.0, vNew, origin, destino, modelNew, time, lNew)), street.size());
                                

            }
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
    
     private void changeToTabay() {
        double spaceMin = SPACEMIN;
        int nvehFromMerida2 = street.size() - 1;
        int nvehToTabay = simcanvas.toTabay.street.size() - 1;
        if (nvehFromMerida2 >= 1) {
            //  System.out.println(" in intersecctionCheck FromMerida: (nvehFromTabay < 1 && nvehFromMerida >= 1)");
            me = (Moveable) (street.elementAt(1));
            double x = me.position();
            double gapFM = laneLength - x - me.length();
            if ((gapFM >= 0 && gapFM <= 2)) {

                if (nvehToTabay >= 1) {
                    lastToTabay = (Moveable) simcanvas.toTabay.street.elementAt(nvehToTabay - 1);

                    changeFromTo(nvehToTabay, nvehFromMerida2, spaceMin, me, lastToTabay, this, simcanvas.toTabay);
                } else {
                    changeFromTo(nvehToTabay, nvehFromMerida2, spaceMin, me, lastToTabay, this, simcanvas.toTabay);
                }
            }
        }// fin si solo veh desde Merida
    }
     
     public void changeFromTo(int nvehTo, int nvehFrom, double spaceMin, Moveable from, Moveable to, MicroStreet streetFrom, MicroStreet streetTo) {
        double vel = 11.; // para la velocidad de To(Valle, Tabay and Merida)
        if (nvehTo == 0) {
            double atime = from.arriveTime();
            from.setArriveTime(atime);
            from.setLane(0);
            from.setPosition(0);

            // velocidad
            MicroModel model = from.model();
            double vNew;
            if (model == idmCar) {
                vNew = idmCar.Veq(vel);// asigna la velocidad
            } else {
                vNew = idmTruck.Veq(vel);
            }
            from.setVelocity(vNew);

            streetTo.street.insertElementAt(from, nvehTo);
            streetFrom.street.removeElementAt(1);
            nvehTo++;
            nvehFrom--;
            return;
        } else if (nvehTo >= 1) {
            to = (Moveable) (streetTo.street.elementAt(nvehTo - 1));
            double posFront = to.position();// posicion;

            if (posFront > spaceMin - 10) {// si esa posicion es mayor que el espacio minimo entonces se inserta
                double atime = from.arriveTime();
                from.setArriveTime(atime);
                from.setPosition(0);

                // velocidad
                MicroModel model = from.model();
                double vNew;
                if (model == idmCar) {
                    vNew = idmCar.Veq(vel);// asigna la velocidad
                } else {
                    vNew = idmTruck.Veq(vel);
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

