/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author Administrador
 */
public class IDMTruckTV extends IDM implements MicroModel, Constants{
    public IDMTruckTV(){
        //System.out.println("in Cstr of class IDMSyncdown (no own ve calc)");

        v0=30/3.6;
        delta=4.0;
        a=A_INIT_TRUCK_MSII;
        b=4.0;
        s0=2.0;
        T=1.7;
        sqrtab=Math.sqrt(a*b);
	initialize();
    }
    
}
