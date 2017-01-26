package de.trafficsimulation;
import java.awt.Color;


public class BCCar implements Moveable{

    private double pos;
    private double vel=10.0;
    private int lane=0;
    private Color color=Color.red;
    private double length;
    private MicroModel model;
    private int dest; // destino desde Merida hacia: El Valle=0, Tabay=1
    private int pri; // prioridad de paso: no pasar=0, pasar=1
    private int origin; // procedencia del vehiculo: 1:FM, 2:FT, 3:FV
    // control de estadisticas
    private double arriveTime;
    private double exitTime;

    public BCCar (double x, double v, int lane, MicroModel model,
                double length){
	pos=x;
	vel=v;
	this.lane=lane;
	this.model=model;
	this.length=length;
    }

    public void setPosition(double x){pos=x;}
    public void setVelocity(double v){vel=v;}
    public void setLane(int lane){this.lane=lane;}
    public void setLength(double length){this.length=length;}
    public void setColor(Color color){this.color=color;}
    public void setModel(MicroModel model){this.model=model;}
    public void setLaneChange(LaneChange lanechange){}
    // Control de estadisticas
    public void setArriveTime(double arriveTime){}
    public void setExitTime(double exitTime){}
    public void setPriority(int prior){}

    public double position(){return pos;}
    public int lane(){return lane;}
    public double velocity(){return vel;}
    public MicroModel model(){return model;}
    public double length(){return length;}
    public Color color(){return Color.white;}
    public int NR(){return 0;}
    public int priority(){return pri;}
    public int destino(){return dest;}
    public int origin(){return origin;}
    // Control de estadisticas
    public double arriveTime(){return arriveTime;}
    public double exitTime(){return exitTime;}

    public boolean timeToChange(double dt){return false;}

    public double dAcc(Moveable vwd, Moveable bwd){
	return 0.0;}

    public boolean change(Moveable fOld, Moveable fNew, Moveable bNew){
	return false;
    }
    public void translate(double dt){}
    public void accelerate(double dt, Moveable vwd){}
    public double acceleration(Moveable vwd){return 0.0;}
}
