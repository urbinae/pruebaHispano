package de.trafficsimulation;

public class IDMTruck extends IDM implements MicroModel, Constants{

    public IDMTruck(){ //parametros por defecto de IDMTruck
	//    System.out.println("in Cstr of class IDMTruck (no own ve calc)");

        v0=9.72;//=35Km/hr = 35/3.6 m/s = 9,72 m/s
        delta=4.0;
        a=A_INIT_TRUCK_MSII;
        b=4.0;
        s0=2.0;
        T=1.7;
        sqrtab=Math.sqrt(a*b);
	initialize(); // de donde proviene este metodo. de IDM
    }
}

