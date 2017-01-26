/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author eimar
 */
public class IDMSyncTruckFT extends IDM implements MicroModel{

    public IDMSyncTruckFT(){
        System.out.println("in Cstr of IDMTruckSyncdown (no own ve calc)");

        v0=25/3.6;
        delta=4.0;
        a=0.1;
        b=2;
        s0=2.0;
        T=2.0;
        sqrtab=Math.sqrt(a*b);
	initialize();
    }
}
