/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author Administrador
 */
public class IDMCarTV extends IDM implements MicroModel, Constants{
    public IDMCarTV(){
        System.out.println("in Cstr of class IDMSyncdown (no own ve calc)");

        v0=60/3.6;
        delta=4.0;
        a=A_INIT_CAR_MSII;  //0.5
	b=B_INIT_MSII;  //3.0
	s0=S0_INIT_M;
	T =T_INIT_S;  //1.5
        sqrtab=Math.sqrt(a*b);
	initialize();
    }
    
}
