using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AircraftMovement : MonoBehaviour {

	public float thrust = 30;
	public float dragCoeff = 0.04f;
	public float lc0 = 0.0047f;
    public float turningSpeed = 50;
    public float gravity = 10;

    float elevatorU = 0;
    float elevatorD = 0;
    float AileronL = 0;
    float AileronR = 0;
    float rudderL = 0;
    float rudderR = 0;

    public bool debug = true;

    private float roh = 1.225f; //dencity of air at sea level and at 15degC

	Rigidbody rb;

    void Start() {
		rb = GetComponent<Rigidbody> ();
        rb.drag = 1;
    }

	void Update()
	{
		if(Input.GetKey("s")){
            if(elevatorU < 300) {
                elevatorU += 0.1f;
            }

        } else {
            elevatorU = 0;
        }

        if(Input.GetKey("w")) {
            if(elevatorD < 300) {
                elevatorD += 0.1f;
            }
        } else {
            elevatorD = 0;
        }

        if(Input.GetKey("a")) {
            rudderL += 0.1f;
        } else {
            rudderL = 0;
        }
        
        if(Input.GetKey("d")) {
            rudderR += 0.1f;
        } else {
            rudderR = 0;
        }

        if(Input.GetKey("z")) {
            AileronL += 0.1f;
        } else {
            AileronL = 0;
        }

        if(Input.GetKey("x")) {
            AileronR += 0.1f;
        } else {
            AileronR = 0;
        }
    }
	void FixedUpdate() {
        calculateLift();
        calculateControlTorque();
    }

	private void calculateLift () {

        Vector3 vel = rb.velocity;
        Vector3 localVel = transform.InverseTransformDirection(vel);
        Vector3 dragDirection   = -rb.velocity.normalized;
        Vector3 thrustDirection = transform.forward;
		Vector3 liftDirection   = Vector3.Cross(-dragDirection, transform.right);

        float angleOfAttack   = Mathf.Atan2(-localVel.y, localVel.z);
		float liftCoeff = lc0 + 2*Mathf.PI*angleOfAttack;
		//float drag = (0.5f) * Mathf.Abs(roh) * Mathf.Abs(dragCoeff) * vel.sqrMagnitude;
        float lift = (0.5f) * Mathf.Abs(roh) * liftCoeff * vel.sqrMagnitude;

        rb.AddForce(thrustDirection * thrust);
        //rb.addforce(dragdirection * drag);
        rb.AddForce(liftDirection	* lift);
    }

    void calculateControlTorque() {
        Vector3 vel = rb.velocity;
        Vector3 localVel = transform.InverseTransformDirection(vel);

        float angleOfAttack = Mathf.Atan2(-localVel.y, localVel.z);
        //float pitchRestoreTorque = (0.5f) * Mathf.Abs(roh) * Mathf.Abs(dragCoeff) * vel.sqrMagnitude * angleOfAttack;

        float elevatorTorque = (elevatorD - elevatorU);
        float aileronTorque = (AileronL - AileronR);
        float rudderTorque = (rudderR - rudderL);

        rb.AddTorque(transform.right * elevatorTorque);
        rb.AddTorque(transform.forward * aileronTorque);
        rb.AddTorque(transform.up * rudderTorque);

        if(debug) {
            DrawArrow.ForDebug(rb.position, new Vector3(localVel.x, -localVel.y, 0), new Color(1, 0, 0, 1));
            DrawArrow.ForDebug(rb.position, new Vector3(localVel.x, 0, localVel.z), new Color(1, 0, 0, 1));
            DrawArrow.ForDebug(rb.position, new Vector3(localVel.x, -localVel.y, localVel.z), new Color(0, 0, 1, 1));
        }
    }
}