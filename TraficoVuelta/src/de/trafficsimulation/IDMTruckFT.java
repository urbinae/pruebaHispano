/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author eimar
 */
public class IDMTruckFT extends IDM implements MicroModel, Constants {

    public IDMTruckFT() {
        v0 = 30 / 3.6;
        delta = 4.0;
        a = 0.4;
        b = 4.;
        s0 = S0_INIT_M;
        T = 1.7;
        sqrtab = Math.sqrt(a * b);
        initialize();
    }
}
