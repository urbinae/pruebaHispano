package de.trafficsimulation;
import java.awt.Color;


public class Car implements Moveable{

    private double vel=10.0;
    private double pos;
    private int    lane=0;
    private Color  color=Color.red;
    private double length=5.;
    private int    nr;  // with nr possible to overwrite default color
                        // in SimCanvas, methods paint*
    private int dest; // destino desde Merida hacia: El Valle=0, Tabay=1
    private int prior; // prioridad de paso: no pasar=0, pasar=1
    private int origin; // procedencia del vehiculo: 1:FM, 2:FT, 3:FV
    // control de estadisticas
    private double arriveTime;
    private double exitTime;
    
    public double tdelay=0.0;
    public double Tdelay=1.6;
    

    public MicroModel model;
    public LaneChange lanechange;
    public double timeServ;
    
    // first state var x,v,lane; then models model,lanechange, then
    // proprties fixed for each car: color, (length,) index n

    public Car (double x, double v, int lane, int ori,
                MicroModel model, LaneChange lanechange, 
                double length, Color color, int number){
        origin = ori;
	pos = x;
	vel = v;
	this.lane = lane;
	this.model = model;
	this.lanechange = lanechange;
        this.length = length;
        this.color = color;
	nr = number;
    }
    // From Valle
    public Car (double x, double v, int lane, int ori,
                MicroModel model, LaneChange lanechange, double arriveT,
                double length){
        origin = ori;
	pos = x;
	vel = v;
	this.lane = lane;
	this.model = model;
	this.lanechange = lanechange;
        this.arriveTime=arriveT;
        this.length = length;
        this.color = color;
	//nr = number;
    }
     
     //inFlow From Merida: con destino el valle, asigno prioridad de paso
     public Car (double x, double v, int ori, int to, int priority,
                MicroModel model, double arriveT, double length){
        System.out.println(" ///////////////////////////////////En Car: From merida1: "+arriveT);
        dest=to;
        prior=priority;
        origin=ori;
	pos = x;
	vel = v;
	this.model = model;
	this.arriveTime=arriveT;
        this.length = length;
        //this.color = color;
    }
     
     // inFlow From Merida: sin prioridad para los vehiculos con destino Tabay
     public Car (double x, double v,int origen, int destino, MicroModel model, double arriveT, 
                double length){
        System.out.println(" ///////////////////////////////////En Car From merida2: "+arriveT);
        dest=destino;
        origin=origen;
	pos = x;
	vel = v;
	this.model = model;
	this.arriveTime=arriveT;
        this.length = length;
        //this.color = color;
    }
    //InFlow From Tabay: con prioridad y Modelo Cambio de canal
    public Car (double x, double v, int lane, int ori, int priority, LaneChange lanechange, 
                MicroModel model, double arriveT, double length){
        System.out.println(" ///////////////////////////////////En Car: "+arriveT);
        prior=priority;
        origin=ori;
	pos = x;
	vel = v;
	this.lane = lane;
	this.model = model;
	this.arriveTime=arriveT;
        this.length = length;
        this.color = color;
    }
    
    public Car (Car carToCopy){
	this.pos = carToCopy.pos;
	this.vel = carToCopy.vel;
	this.lane = carToCopy.lane;
	this.model = carToCopy.model;
	this.lanechange = carToCopy.lanechange;
        this.length = carToCopy.length;
        this.color = carToCopy.color;
	this.nr = carToCopy.nr;
    }
    
    public Car(double x, int lane, double v){
        pos = x;
        vel = v;
        this.lane = lane;
    }

    public void setPosition(double x){pos=x;}
    public void setVelocity(double v){vel=v;}
    public void setLane(int lane){this.lane=lane;}
    public void setModel(MicroModel model){this.model=model;}
    public void setLaneChange(LaneChange lanechange){
         this.lanechange=lanechange;}
    public void setLength(double length){this.length=length;}
    public void setColor(Color color){this.color=color;}
    public void setColorTV(Color color){this.color=color;}
    public void setColorTT(Color color){this.color=color;}
    // Control de estadisticas
    public void setArriveTime(double arriveTime){}
    public void setExitTime(double exitTime){}
    public void setPriority(int prior){this.prior = prior;}
    
    
    public double position(){return pos;}
    public double velocity(){return vel;}
    public int    lane(){return lane;}
    public MicroModel model(){return model;}
    public double length(){return length;}
    public Color color(){return color;}
    public int NR(){return nr;}
    public int priority(){return prior;}
    public int destino(){return dest;}
    public int origin(){return origin;}
    // Control de estadisticas
    public double arriveTime(){return arriveTime;}
    public double exitTime(){return exitTime;}

    public boolean timeToChange(double dt){
	tdelay=tdelay+dt;
	if (tdelay>=Tdelay){
	   tdelay=tdelay-Tdelay;
	   return true;
	}
	else {return false;}
    }


    public double dAcc(Moveable vwd, Moveable bwd){

	double b=bwd.acceleration(this);
	double a=bwd.acceleration(vwd);
	return (b-a);
    }
    public boolean change(Moveable fOld, Moveable fNew, Moveable bNew){
	System.out.println("in Car.change: nr="+nr);
	return lanechange.changeOK(this, fOld, fNew, bNew);
    }

    public void translate(double dt){
	pos=pos+dt*vel;
    }
    public void accelerate(double dt, Moveable vwd){
        //Llamada del metodo calcAcc de la interfaz MicroModel implementado en la clase IDM
	double acc=model.calcAcc(this, vwd);
	vel=vel+acc*dt;
	if (vel<0.0){
	    //double Out1=new Double(vwd.velocity());
	    vel=0.0;
	}
    }
    public double acceleration(Moveable vwd){
	double acc=model.calcAcc(this, vwd);
	return acc;
    }
}
