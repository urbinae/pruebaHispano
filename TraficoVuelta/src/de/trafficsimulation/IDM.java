package de.trafficsimulation;



/**
Basis class for the microscopic traffic model IDM 
(intelligent-driver model, 
see <a href="http://xxx.uni-augsburg.de/abs/cond-mat/0002177">
M. Treiber, A. Hennecke, and D. Helbing, Congested Traffic States in Empirical Observations and Microscopic Simulations, 
Phys. Rev. E 62, 1805 (2000)].</a>
<br><br>
    The classes IDMCar, IDMTruck, etc are concrete realisations 
of this class for trucks, cars, etc. 

*/

public abstract class IDM implements MicroModel{



    public double v0;//velocidad deseada
    public double delta; // coeficiente de agresividad
    public double a; // aceleracion
    public double b; // desaceleracion
    public double s0; // brecha minima
    public double s1; // distancia deseada, si s0=s1 no hay frenado, la ruta es libre
    public double T; /*El tiempo promedio de avance.
                    Recomendación: 1,8 s; los valores reales varían entre 2 s y 0,8 s, e incluso por debajo.*/
    public double sqrtab;
    private static final int ismax=100;          // ve(s)=ve(ismax) for s>ismax assumed
    private double[] veqTab = new double[ismax+1]; // table in steps of ds=1m // tabla en pasos de ds=1m

    public IDM(){; } //parametros por defecto de IDM

    public void set_v0(double v0){this.v0=v0;}
    public void set_T(double T){this.T=T;}
    public void set_a(double a){this.a=a;}
    public void set_b(double b){this.b=b;}
    public void set_s0(double s0){this.s0=s0;}
    public void set_s1(double s1){this.s1=s1;}

    public void set_params(IDM idm){
	this.set_v0(idm.v0); //
	this.set_a(idm.a);
	this.set_b(idm.b);
	this.set_T(idm.T);
	this.set_s0(idm.s0);
	this.set_s1(idm.s1);
    }

    // calculate table of equilibrium velocity (s) with relaxation method;
    // veqTab[i] = equilibrium velocity for s=i*ds with ds=1 m, i=0..100
    // calculo de la tabla de velocidad de equilibrio para s=i*ds, con ds=1m , i desde 0 a 100m

    public void initialize(){
        final double dt=0.5;             // relaxation with timestep=0.5 s,tiempo del paso = 0.5
        final double kmax=20;            // number of iterations in rlaxation
	veqTab[0]=0.0;
	for (int is=1; is<=ismax ; is++){     // table in steps of ds=1 (m), para is <= 100

	    double Ve=veqTab[is-1]; // parte desde 0 a 99
            // if(is>=ismax-3){System.out.println("is="+is+" Ve="+Ve);}

	    for (int k=0; k<kmax; k++){ // para k < 20
		double s_star = s0 + s1*Math.sqrt(Ve/v0)+Ve*T; // 	    
		double acc= a * (1 - Math.pow((Ve/v0),delta) - (s_star*s_star)/(is*is) );
		Ve=Ve+acc*dt; // 
		if (Ve<0.0){Ve=0.0;}
	    }
	    veqTab[is]=Ve;
            //veqTab[is]=Ve;
            //System.out.println("IDM.initialize():"
        //+ "  veqTab["+is+"]= "+veqTab[is]);
	}
        System.out.println("IDM.initialize():"
        + "  veqTab[0]="+veqTab[0]
        + ", veqTab["+ismax+"]="+veqTab[ismax]
        );
	
    }

    // function for equilibrium velocity using above table; ve(s>ismax)=ve(ismax)
 
    public double Veq(double dx){ 
	int is = (int) dx;
	double V=0.0;
	if (is<ismax){ // si 5 < 100 
	    double rest=dx-((double) is); // = 5.-5. = 0
	    V = (1-rest)*veqTab[is] + rest*veqTab[is+1]; // = (1+0)*veqTab[5]+(-0)*veqTab[6]
	}                                                // =1*1.9702 -0*2.613 = 1.9702
	if (is>=ismax){ V=veqTab[ismax];} // V=veqTab[100] la maxima velocidad
	if (is<=0){V=0.0;}
	return V;
    }

    public double calcAcc(Moveable bwd, Moveable vwd){
        // Diferencia de velocidad
        double delta_v=bwd.velocity()-vwd.velocity();//mi vel - front vel
        double s=vwd.position()-bwd.position()-bwd.length();  // pos: END of vehicles!: brecha entre mi vehiculo y el del frente
        double vel = bwd.velocity();// mi velocidad
        //System.out.println("calcAcc: s0: "+s0+" s1: "+s1);
        double s_star_raw = s0 + s1*Math.sqrt(vel/v0)+vel*T// brecha deseada, s0=2, s1=0
          + (vel*delta_v)/(2*sqrtab);
        double s_star = (s_star_raw > s0) ? s_star_raw : s0;// brecha deseada > 2?
        double acc = a * (1 - Math.pow((vel/v0),delta) -(s_star*s_star)/(s*s) );// acceleracion
        if (acc < -Constants.MAX_BRAKING) {
            acc = -Constants.MAX_BRAKING;
        }
        return acc;	
    }

 }
