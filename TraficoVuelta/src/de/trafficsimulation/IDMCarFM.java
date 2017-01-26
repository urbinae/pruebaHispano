/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.trafficsimulation;

/**
 *
 * @author eimar
 */
public class IDMCarFM extends IDM implements MicroModel, Constants {

    public IDMCarFM() {
        v0 = 25 / 3.6;
        delta = 4.0;
        a = A_INIT_CAR_MSII;  //0.5
        b = B_INIT_MSII;  //3.0
        s0 = S0_INIT_M;
        T = T_INIT_S;  //1.5
        sqrtab = Math.sqrt(a * b);
        initialize();
    }
}
