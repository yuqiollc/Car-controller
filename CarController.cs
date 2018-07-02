using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{
    #region public variables

    public Car _Car;
    public DefaultCar _DefaultCar;

    /// <summary>
    /// Engine Power for Transmission State with curve dependency
    /// </summary>
    public AnimationCurve[] EngineTorqueCurve;
    /// <summary>
    /// Speed required to change Transmission state
    /// </summary>
    public float[] TargetSpeedForGear;
    /// <summary>
    /// Maximum speed for current Transmission State
    /// </summary>
    public float[] MaxSpeedForGear;
    /// <summary>
    /// Current RPM
    /// </summary>
    public float EngineRPM = 0f;
    /// <summary>
    /// Coherent RPM
    /// </summary>
    public float RawEngineRPM = 0f;
    [HideInInspector]
    /// <summary>
    /// Forward torque
    /// </summary>
    public float EngineTorque = 2000f;
    /// <summary>
    /// Forward/Back button handle [-1, 1]
    /// </summary>
    [HideInInspector]
    public float CurrentAccel;
    /// <summary>
    /// Back button handle [0, 1]
    /// </summary>
    [HideInInspector]
    public float CurrentFootbrake;

    /// <summary>
    /// True if trailer is connected
    /// </summary>
    public bool IsConnected = false;

    /// <summary>
    /// Current car speed
    /// </summary>
    public double Speed { get { return _car_Rigidbody.velocity.magnitude * 3.6; } }
    /// <summary>
    /// Forward Speed
    /// </summary>
    public double CurrentDirectionSpeed
    {
        get
        {
            if (transform.InverseTransformDirection(_car_Rigidbody.velocity).z > 0)
                return Speed;
            else return -Speed;
        }
    }
    /// <summary>
    /// Current Transmission State
    /// </summary>
    public int CurrentGear { get; private set; }
    /// <summary>
    /// [0,1] Is Accelerator Active
    /// </summary>
    public float GasInput
    {
        get
        {
            if (CurrentAccel > 0)
                return CurrentAccel;
            if (CurrentAccel < 0)
                return -1 * CurrentAccel;

            return 0;
        }
    }

    #endregion

    private void Awake()
    {
        _car_Rigidbody = GetComponent<Rigidbody>();
    }

    // Use this for initialization
    void Start()
    {
        SetDefaultValues();

        if(FindObjectOfType<User>())
            RecalcCarCharacteristics();

        SetWheels();
        SetWheelMass();
        SetCOM();
        TorqueCurve();
    }

    // Update is called once per frame
    void Update()
    {
        SyncWheelsAndColliders();

        if (GasInput >= .1f)
            _launched += GasInput * Time.deltaTime;
        else
            _launched -= Time.deltaTime;

        _launched = Mathf.Clamp(_launched, 0f, 1f);
    }

    /// <summary>
    /// Wheels mass handler with Current Car weight dependence
    /// </summary>
    private void SetWheelMass()
    {
        foreach (var wc in _wheelColliders)
        {
            wc.mass = _car_Rigidbody.mass / 15f;
        }
    }

    /// <summary>
    /// Find Colliders and Meshes for wheels
    /// </summary>
    private void SetWheels()
    {
        GameObject wheels = null;
        GameObject wColliders = null;

        //Find containers for Meshes and Colliders
        for (int i = 0; i < gameObject.transform.childCount; i++)
        {
            if (gameObject.transform.GetChild(i).CompareTag("Wheels"))
                wheels = gameObject.transform.GetChild(i).gameObject;
            if (gameObject.transform.GetChild(i).CompareTag("WheelHubs"))
                wColliders = gameObject.transform.GetChild(i).gameObject;
        }

        if (wheels == null)
        {
            Debug.Log("Object with tag `Wheels` not found");
            return;
        }

        if (_wheelColliders == null)
        {
            Debug.Log("Object with tag `WheelHubs` not found");
            return;
        }

        for (int i = 0; i < wheels.transform.childCount; i++)
        {
            _wheelMeshes.Add(wheels.transform.GetChild(i).gameObject);
        }

        for (int i = 0; i < wColliders.transform.childCount; i++)
        {
            _wheelColliders.Add(wColliders.transform.GetChild(i).GetComponent<WheelCollider>());
        }
    }

    /// <summary>
    /// True if Car Speed is higher than Maximum Speed
    /// </summary>
    /// <returns></returns>
    bool OverTorque()
    {

        if (Speed > _maxSpeed)
            return true;

        return false;
    }

    /// <summary>
    /// Pushing the wheel forward
    /// </summary>
    /// <param name="wc">Wheel</param>
    /// <param name="torque">Power</param>
    void ApplyMotorTorque(WheelCollider wc, float torque)
    {
        WheelHit hit;
        wc.GetGroundHit(out hit);

        if (Mathf.Abs(wc.rpm) >= 100)
        {
            if (hit.forwardSlip > .25f)
            {
                torque -= Mathf.Clamp(torque * (hit.forwardSlip) * _TCSStrength, 0f, EngineTorque);
            }
            else
            {
                torque += Mathf.Clamp(torque * (hit.forwardSlip) * _TCSStrength, -EngineTorque, 0f);
            }
        }

        if (OverTorque())
            torque = 0;

        wc.motorTorque = ((torque * (1 - _clutchInput) * 1) * GasInput) * (EngineTorqueCurve[CurrentGear].Evaluate((float)Speed) * _direction);

        //DebugInfo.instance.Add("motorTorque", wc.motorTorque);
        //DebugInfo.instance.Add("EngineTorqueCurve: ", EngineTorqueCurve[CurrentGear].Evaluate((float)Speed));
        //ApplyEngineSound(wheelCollider.motorTorque);
    }

    /// <summary>
    /// Center Mass for Car
    /// </summary>
    private void SetCOM()
    {
        _com = transform.Find("CenterOfMass");
        if (_com == null)
            print("Not found `CenterOfMass` in " + gameObject.name);

        _car_Rigidbody.centerOfMass = _com.localPosition;
    }

    /// <summary>
    /// Mesh and wheel rotation synchronization
    /// </summary>
    private void SyncWheelsAndColliders()
    {
        for (int i = 0; i < _wheelColliders.Count; i++)
        {
            Quaternion quat;
            Vector3 position;
            _wheelColliders[i].GetWorldPose(out position, out quat);
            _wheelMeshes[i].transform.position = position;
            _wheelMeshes[i].transform.rotation = quat;
        }
    }

    /// <summary>
    /// Forward wheels rotation
    /// </summary>
    /// <param name="steer"></param>
    private void SetSteer(float steer)
    {
        steer = Mathf.Clamp(steer, -1, 1);

        // Define wheels 0 and 1 as Forward Wheels
        float steerAngle = steer * _maximimSteerAngle;
        _wheelColliders[0].steerAngle = steerAngle;
        _wheelColliders[1].steerAngle = steerAngle;
    }

    /// <summary>
    /// Car Movement
    /// </summary>
    /// <param name="accel">Forward/Backward</param>
    /// <param name="footbrake">Brake</param>
    /// <param name="forceEngine">True if Forced mode is ON</param>
    private void ApplyDrive(float accel, float footbrake, bool forceEngine)
    {
        float fuelConsumption = _DefaultCar.FuelConsumption;

        if (IsConnected)
            fuelConsumption = fuelConsumption * 2;

        if (forceEngine)
            _Car.Fuel -= fuelConsumption * 2;
        else if(accel > 0)
            _Car.Fuel -= fuelConsumption;
        else
            _Car.Fuel -= fuelConsumption / 10;

        if (_Car.Fuel < 0)
            _Car.Fuel = 0;

        //Brake for each wheel
        float thrustTorque = (_brakeTorque / _wheelColliders.Count);

        //DebugInfo.Add("accel", accel);

        //Braking process
        if (CurrentDirectionSpeed > 3 && accel < 0)
            ApplyBrake(Mathf.Abs(accel) * thrustTorque);
        else if (CurrentDirectionSpeed < -3 && accel > 0)
            ApplyBrake(Mathf.Abs(accel) * thrustTorque);
        else
            ApplyBrake(0);

        //Torque per each wheel
        thrustTorque = (EngineTorque / _wheelColliders.Count);
        if (accel >= 0 || accel <= 0)
        {
            for (int i = 0; i < _wheelColliders.Count; i++)
            {
                ApplyMotorTorque(_wheelColliders[i], (forceEngine) ? (accel * thrustTorque * _forceEngine) : (accel * thrustTorque));
            }
        }
    }

    /// <summary>
    /// Set Brake
    /// </summary>
    /// <param name="brake"></param>
    private void ApplyBrake(float brake)
    {
        for (int i = 0; i < _wheelColliders.Count; i++)
        {
            _wheelColliders[i].brakeTorque = brake;
            DebugInfo.Add("Back", _wheelColliders[i].brakeTorque);
        }
    }

    /// <summary>
    /// Steer Helper on road turns
    /// </summary>
    private void SteerHelper()
    {
        for (int i = 0; i < _wheelColliders.Count; i++)
        {
            WheelHit wheelhit;
            _wheelColliders[i].GetGroundHit(out wheelhit);
            if (wheelhit.normal == Vector3.zero)
                return; // Wheels are not on earth, no changes in speed
        }

        // provide support if car turning angle is too big
        if (Mathf.Abs(_oldRotation - transform.eulerAngles.y) < 10f)
        {
            var turnadjust = (transform.eulerAngles.y - _oldRotation) * _steeerHelper;
            Quaternion velRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
            _car_Rigidbody.velocity = velRotation * _car_Rigidbody.velocity;
        }
        _oldRotation = transform.eulerAngles.y;
    }

    /// <summary>
    /// Engine Momentum calculation
    /// </summary>
    private void Engine()
    {
        if (_Car.Fuel == 0)
        {
            EngineRPM = 0;
        }

        float wheelRPMToSpeedL = (((_wheelColliders[0].rpm * _wheelColliders[0].radius) / 2.9f)) * _car_Rigidbody.transform.lossyScale.y;
        float wheelRPMToSpeedR = (((_wheelColliders[1].rpm * _wheelColliders[1].radius) / 2.9f)) * _car_Rigidbody.transform.lossyScale.y;

        float wheelRPM = wheelRPMToSpeedR + wheelRPMToSpeedL;

        RawEngineRPM = Mathf.Clamp(Mathf.MoveTowards(RawEngineRPM, (_maxEngineRPM * 1.1f) *
            (Mathf.Clamp01(Mathf.Lerp(0f, 1f, (1f - _clutchInput) *
                (((wheelRPM * _direction) / 2f) / MaxSpeedForGear[CurrentGear])) +
                (((GasInput) * _clutchInput) + _idleInput)))
                                                     , _engineInertia * 100f), 0f, _maxEngineRPM * 1.1f);

        EngineRPM = Mathf.Lerp(EngineRPM, RawEngineRPM, Mathf.Lerp(Time.fixedDeltaTime, Time.fixedDeltaTime * 100f, RawEngineRPM / _maxEngineRPM * 0.001f));
    }

    private void Cluch()
    {
        _idleInput = Mathf.Lerp(1f, 0f, EngineRPM / _minEngineRPM);

        if (CurrentGear == 0)
        {
            float wheelRPMToSpeedL = (((_wheelColliders[0].rpm * _wheelColliders[0].radius) / 2.9f)) * _car_Rigidbody.transform.lossyScale.y;
            float wheelRPMToSpeedR = (((_wheelColliders[1].rpm * _wheelColliders[1].radius) / 2.9f)) * _car_Rigidbody.transform.lossyScale.y;

            if (_useClutchMarginAtFirstGear)
            {
                if (_launched >= .25f)
                    _clutchInput = Mathf.Lerp(_clutchInput, (Mathf.Lerp(1f, (Mathf.Lerp(_clutchInertia, 0f, ((wheelRPMToSpeedL + wheelRPMToSpeedR) / 2f) / TargetSpeedForGear[0])), Mathf.Abs(GasInput))), Time.fixedDeltaTime * 5f);
                else
                    _clutchInput = Mathf.Lerp(_clutchInput, 1f, Time.fixedDeltaTime * 5f);
            }
            else
            {
                _clutchInput = Mathf.Lerp(_clutchInput, (Mathf.Lerp(1f, (Mathf.Lerp(_clutchInertia, 0f, ((wheelRPMToSpeedL + wheelRPMToSpeedR) / 2f) / TargetSpeedForGear[0])), Mathf.Abs(GasInput))), Time.fixedDeltaTime * 5f);
            }
        }
        else
        {
            if (_gearIsChanging)
                _clutchInput = Mathf.Lerp(_clutchInput, 1, Time.fixedDeltaTime * 5f);
            else
                _clutchInput = Mathf.Lerp(_clutchInput, 0, Time.fixedDeltaTime * 5f);
        }


        _clutchInput = Mathf.Clamp01(_clutchInput);
    }

    /// <summary>
    /// Transition state change with speed dependency
    /// </summary>
    private void GearBox()
    {
        if (CurrentGear < _totalGears - 1 && !_gearIsChanging)
        {
            if (Speed >= (TargetSpeedForGear[CurrentGear]) && _wheelColliders[0].rpm > 0)
            {
                if (_direction != -1)
                    StartCoroutine(ChangingGearCoroutine(CurrentGear + 1));
            }
        }

        if (CurrentGear > 0)
        {
            if (!_gearIsChanging)
            {
                if (Speed < (TargetSpeedForGear[CurrentGear - 1] * .8f) && _direction != -1)
                {
                    StartCoroutine(ChangingGearCoroutine(CurrentGear - 1));
                }
            }
        }
    }

    /// <summary>
    /// Change of transition state
    /// </summary>
    /// <param name="gear"></param>
    /// <returns></returns>
    internal IEnumerator ChangingGearCoroutine(int gear)
    {
        _gearIsChanging = true;

        yield return new WaitForSeconds(_gearShiftingDelay);

        if (gear == -1)
        {
            CurrentGear = 0;
            _direction = -1;
        }
        else
        {
            CurrentGear = gear;
            _direction = 1;
        }

        _gearIsChanging = false;
    }

    /// <summary>
    /// Speed limit and Power limit for current Transition state
    /// </summary>
    private void TorqueCurve()
    {
        //Maximum speed for each transition
        if (MaxSpeedForGear == null)
            MaxSpeedForGear = new float[_totalGears];

        if (TargetSpeedForGear == null)
            TargetSpeedForGear = new float[_totalGears - 1];

        if (MaxSpeedForGear != null && MaxSpeedForGear.Length != _totalGears)
            MaxSpeedForGear = new float[_totalGears];

        if (TargetSpeedForGear != null && TargetSpeedForGear.Length != _totalGears - 1)
            TargetSpeedForGear = new float[_totalGears - 1];

        for (int j = 0; j < _totalGears; j++)
            MaxSpeedForGear[j] = Mathf.Lerp(0f, _maxSpeed * 1.1f, (float)(j + 1) / (float)(_totalGears));
        for (int k = 0; k < _totalGears - 1; k++)
            TargetSpeedForGear[k] = Mathf.Lerp(0, _maxSpeed * Mathf.Lerp(0f, 1f, _gearShiftingThreshold), ((float)(k + 1) / (float)(_totalGears)));

        EngineTorqueCurve = new AnimationCurve[_totalGears];

        //CurrentGear = 0;

        for (int i = 0; i < EngineTorqueCurve.Length; i++)
        {
            EngineTorqueCurve[i] = new AnimationCurve(new Keyframe(0, 1));
        }

        for (int i = 0; i < _totalGears; i++)
        {

            if (i != 0)
            {
                EngineTorqueCurve[i].MoveKey(0, new Keyframe(0, Mathf.Lerp(1f, .05f, (float)(i + 1) / (float)_totalGears)));
                EngineTorqueCurve[i].AddKey(Mathf.Lerp(0, _maxSpeed * .75f, ((float)(i) / (float)(_totalGears))), Mathf.Lerp(1f, .5f, ((float)(i) / (float)(_totalGears))));
                EngineTorqueCurve[i].AddKey(Mathf.Lerp(0, _maxSpeed * 1.25f, ((float)(i + 1) / (float)(_totalGears))), .05f);
                EngineTorqueCurve[i].AddKey(Mathf.Lerp(0, _maxSpeed, ((float)(i + 1) / (float)(_totalGears))) * 2f, -3f);
                EngineTorqueCurve[i].postWrapMode = WrapMode.Clamp;
            }
            else
            {
                EngineTorqueCurve[i].MoveKey(0, new Keyframe(0, 2f));
                EngineTorqueCurve[i].AddKey(MaxSpeedForGear[i] / 5f, 2.5f);
                EngineTorqueCurve[i].AddKey(MaxSpeedForGear[i], 0f);
                EngineTorqueCurve[i].postWrapMode = WrapMode.Clamp;
            }
        }
    }

    /// <summary>
    /// Car Controller
    /// </summary>
    /// <param name="steering">Horizontal control</param>
    /// <param name="accel">Torque</param>
    /// <param name="footbrake">Vertical footbrake</param>
    /// <param name="forceEngine">True if Forced mode is ON</param>
    public void Move(float steering, float accel, float footbrake, bool forceEngine)
    {
        //DebugInfo.Add("CurrentDirectionSpeed", (float)CurrentDirectionSpeed);

        // Limitation of input values
        CurrentFootbrake = footbrake = Mathf.Clamp(footbrake, 0, 1);
        CurrentAccel = accel = Mathf.Clamp(accel, -1, 1);


        //DebugInfo.instance.Add("accel", accel);

        if (_Car.Fuel > 0)
            ApplyDrive(accel, footbrake, forceEngine);
        SteerHelper();

        Engine();
        GearBox();
        Cluch();


        SetSteer(steering);
    }

    /// <summary>
    /// Stop the car
    /// </summary>
    public void StopCar()
    {
        StartCoroutine(StopCarCoroutine());
    }

    /// <summary>
    /// Car is having Brake applied
    /// </summary>
    /// <returns></returns>
    private IEnumerator StopCarCoroutine()
    {
        while (Speed > .1f)
        {
            ApplyBrake(_brakeTorque);
            yield return new WaitForFixedUpdate();
        }

        ApplyBrake(0);
    }

    /// <summary>
    /// Recalculations for components with level dependencies
    /// </summary>
    private void RecalcCarCharacteristics()
    {
        if(_Car == null)
        {
            print("_Car is null");
            return;
        }
        _maxSpeed = _Car.MaxSpeedWithLVL;
        EngineTorque = _Car.MaxEngineTorqueWithLVL;
    }

    /// <summary>
    /// Input of car characteristics from server
    /// </summary>
    private void SetDefaultValues()
    {
        _maxSpeed = _DefaultCar.MaxSpeed;
        EngineTorque = _DefaultCar.EngineToeque;
        _brakeTorque = _DefaultCar.BrakeTorque;

        _maximimSteerAngle = _DefaultCar.MaximumSteerAngle;
        _totalGears = _DefaultCar.TotalGears;
        _maxEngineRPM = _DefaultCar.MaxEngineRPM;
        _minEngineRPM = _DefaultCar.MinEngineRPM;
        _gearShiftingDelay = _DefaultCar.GearShiftingDelay;
        _gearShiftingThreshold = _DefaultCar.GearShiftingThreshold;
        _clutchInertia = _DefaultCar.ClutchInertia;
        _engineInertia = _DefaultCar.EngineInertia;
        _TCSStrength = _DefaultCar.TcsStrength;
        _forceEngine = _DefaultCar.ForceEngine;
        _steeerHelper = _DefaultCar.SteerHelper;
        _useClutchMarginAtFirstGear = _DefaultCar.UseClutchMargineAtFirstGear;
    }

    //private void OnDestroy()
    //{
    //    ClientRequest.Request(RequestType.UpdateCarFuel, () => { }, _Car.Fuel);
    //    print("CarController Destroy");
    //}

    private void OnApplicationQuit()
    {
        //ClientRequest.Request(RequestType.UpdateCarFuel, () => { }, false, _Car.Fuel);
    }

    private void OnCollisionEnter(Collision collision)
    {
        TakeDamage(collision.relativeVelocity.magnitude);
    }

    /// <summary>
    /// Car Damage System
    /// </summary>
    /// <param name="damage">Damage</param>
    private void TakeDamage(float damage)
    {
        float health = _Car.Health - damage;
        if (health < 0)
            _Car.Health = 0;
        else
            _Car.Health = health;
    }

    #region private variables

    /// <summary>
    /// Current Reverse state
    /// </summary>
    private int _direction = 1;
    /// <summary>
    /// Number of States for Transition
    /// </summary>
    private int _totalGears = 8;
    /// <summary>
    /// Maximum speed
    /// </summary>
    private float _maxSpeed = 120;

    /// <summary>
    /// Steer Angle
    /// </summary>
    private float _maximimSteerAngle = 20;
    /// <summary>
    /// Brake torque
    /// </summary>
    private float _brakeTorque = 2000f;
    /// <summary>
    /// Maximum value of RPM
    /// </summary>
    private float _maxEngineRPM = 7000f;
    /// <summary>
    /// minimum value of RPM
    /// </summary>
    private float _minEngineRPM = 1000f;
    [Range(0f, .5f)] private float _gearShiftingDelay = .35f;
    [Range(.5f, .95f)] private float _gearShiftingThreshold = .85f;
    [Range(.1f, .9f)] private float _clutchInertia = .25f;
    [Range(.75f, 2f)] private float _engineInertia = 1f;
    [Range(0f, 1f)] private float _TCSStrength = 1f;
    /// <summary>
    /// Value for Forced modeК
    /// </summary>
    [Range(1f, 2f)] private float _forceEngine = 1.3f;
    /// <summary>
    /// 1 - Full assistance on roadturns
    /// </summary>
    [Range(0f, 1f)] private float _steeerHelper = 1f;

    private bool _useClutchMarginAtFirstGear = true;
    private float _idleInput = 0;
    private float _clutchInput = 0;
    private float _launched = 0;

    /// <summary>
    /// Wheels Colliders
    /// </summary>
    [SerializeField]
    private List<WheelCollider> _wheelColliders = new List<WheelCollider>();
    /// <summary>
    /// Wheels Meshes
    /// </summary>
    [SerializeField]
    private List<GameObject> _wheelMeshes = new List<GameObject>();
    /// <summary>
    /// Center mass
    /// </summary>
    [SerializeField]
    private Transform _com;
    /// <summary>
    /// Rigidbody of Car
    /// </summary>
    private Rigidbody _car_Rigidbody;
    /// <summary>
    /// Previous turning angle
    /// </summary>
    private float _oldRotation;
    /// <summary>
    /// True, if now Transition state is changed
    /// </summary>
    private bool _gearIsChanging = false;

    #endregion
}
