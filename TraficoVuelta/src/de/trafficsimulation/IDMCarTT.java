/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author Administrador
 */
public class IDMCarTT extends IDM implements MicroModel, Constants{
    public IDMCarTT(){
        System.out.println("in Cstr of class IDMSyncdown (no own ve calc)");

        v0=100/3.6;
        delta=4.0;
        a=0.5;
        b=3.;
        s0=2.0;
        T=1.5;
        sqrtab=Math.sqrt(a*b);
	initialize();
    }
    
}
