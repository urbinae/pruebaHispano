/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author Administrador
 */
public class IDMSyncCarTM extends IDM implements MicroModel, Constants{
    public IDMSyncCarTM(){
        System.out.println("in Cstr of class IDMSyncdown (no own ve calc)");

        v0=15/3.6;
        delta=4.0;
        a=0.6;
        b=0.9;
        s0=2.0;
        T=2.0;
        sqrtab=Math.sqrt(a*b);
	initialize();
    }
    
}
