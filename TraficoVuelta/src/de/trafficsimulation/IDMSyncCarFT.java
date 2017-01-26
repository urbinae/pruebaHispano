/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author eimar
 */
public class IDMSyncCarFT extends IDM implements MicroModel, Constants{
    public IDMSyncCarFT(){
        //System.out.println("in Cstr of class IDMSyncCarFT (no own ve calc)");

        v0=30/3.6;
        delta=4.0;
        a=0.6;
        b=0.9;
        s0=2.0;
        T=2.0;
        sqrtab=Math.sqrt(a*b);
	initialize();
    }
    
}
