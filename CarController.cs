using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{
    #region public variables

    public Car _Car;
    public DefaultCar _DefaultCar;

    /// <summary>
    /// Мощность на каждую передачу в зависимости от кривой
    /// </summary>
    public AnimationCurve[] EngineTorqueCurve;
    /// <summary>
    /// Скорость при которой будет смена передачи
    /// </summary>
    public float[] TargetSpeedForGear;
    /// <summary>
    /// Максимальная скорость для текущей передачи
    /// </summary>
    public float[] MaxSpeedForGear;
    /// <summary>
    /// Текущий RPM
    /// </summary>
    public float EngineRPM = 0f;
    /// <summary>
    /// Сглаженый RPM
    /// </summary>
    public float RawEngineRPM = 0f;
    [HideInInspector]
    /// <summary>
    /// Крутящий момент вперед
    /// </summary>
    public float EngineTorque = 2000f;
    /// <summary>
    /// Нажатие кнопки вперед/назад [-1, 1]
    /// </summary>
    [HideInInspector]
    public float CurrentAccel;
    /// <summary>
    /// Нажатие кнопки назад [0, 1]
    /// </summary>
    [HideInInspector]
    public float CurrentFootbrake;

    /// <summary>
    /// True - если к машине прикреплен прицеп
    /// </summary>
    public bool IsConnected = false;

    /// <summary>
    /// Текущая скорость машины
    /// </summary>
    public double Speed { get { return _car_Rigidbody.velocity.magnitude * 3.6; } }
    /// <summary>
    /// Скорость движения по направлению вперед
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
    /// Текущая передача
    /// </summary>
    public int CurrentGear { get; private set; }
    /// <summary>
    /// [0,1] Нажата ли педаль газа
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
    /// Установка массы колес в зависимомти от массы машины
    /// </summary>
    private void SetWheelMass()
    {
        foreach (var wc in _wheelColliders)
        {
            wc.mass = _car_Rigidbody.mass / 15f;
        }
    }

    /// <summary>
    /// Находит колайдеры и меши колес у машины
    /// </summary>
    private void SetWheels()
    {
        GameObject wheels = null;
        GameObject wColliders = null;

        //Найти контейнеры с мешами и колайдерами
        for (int i = 0; i < gameObject.transform.childCount; i++)
        {
            if (gameObject.transform.GetChild(i).CompareTag("Wheels"))
                wheels = gameObject.transform.GetChild(i).gameObject;
            if (gameObject.transform.GetChild(i).CompareTag("WheelHubs"))
                wColliders = gameObject.transform.GetChild(i).gameObject;
        }

        if (wheels == null)
        {
            Debug.Log("Не найден обьект с тегом `Wheels`");
            return;
        }

        if (_wheelColliders == null)
        {
            Debug.Log("Не найден обьект с тегом `WheelHubs`");
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
    /// True если скорость машины больше максимальной
    /// </summary>
    /// <returns></returns>
    bool OverTorque()
    {

        if (Speed > _maxSpeed)
            return true;

        return false;
    }

    /// <summary>
    /// Толкане колеса вперед
    /// </summary>
    /// <param name="wc">Колесо</param>
    /// <param name="torque">Мощность</param>
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
    /// Присвоение машине центр масы
    /// </summary>
    private void SetCOM()
    {
        _com = transform.Find("CenterOfMass");
        if (_com == null)
            print("Не удалось найти `CenterOfMass` у " + gameObject.name);

        _car_Rigidbody.centerOfMass = _com.localPosition;
    }

    /// <summary>
    /// Синхронизация оборотов колес с мешем
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
    /// Поворот передних колес машины
    /// </summary>
    /// <param name="steer"></param>
    private void SetSteer(float steer)
    {
        steer = Mathf.Clamp(steer, -1, 1);

        // Предположим, что колеса 0 и 1 являются передними колесами.
        float steerAngle = steer * _maximimSteerAngle;
        _wheelColliders[0].steerAngle = steerAngle;
        _wheelColliders[1].steerAngle = steerAngle;
    }

    /// <summary>
    /// Движение машиной
    /// </summary>
    /// <param name="accel">Вперед/назад</param>
    /// <param name="footbrake">Тормоз</param>
    /// <param name="forceEngine">True если включен форсированый режим</param>
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

        //Торможение на каждое колесо
        float thrustTorque = (_brakeTorque / _wheelColliders.Count);

        //DebugInfo.Add("accel", accel);

        //Торможение
        if (CurrentDirectionSpeed > 3 && accel < 0)
            ApplyBrake(Mathf.Abs(accel) * thrustTorque);
        else if (CurrentDirectionSpeed < -3 && accel > 0)
            ApplyBrake(Mathf.Abs(accel) * thrustTorque);
        else
            ApplyBrake(0);

        //Крутящий момент на каждое колесо
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
    /// Задать торможение
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
    /// Помогает машине не перевернутся при поворотах
    /// </summary>
    private void SteerHelper()
    {
        for (int i = 0; i < _wheelColliders.Count; i++)
        {
            WheelHit wheelhit;
            _wheelColliders[i].GetGroundHit(out wheelhit);
            if (wheelhit.normal == Vector3.zero)
                return; // колеса не на земле, так что не перестраиваем скорость
        }

        // если угол поворота машины достаточно большой, то не даем ей перевернутся
        if (Mathf.Abs(_oldRotation - transform.eulerAngles.y) < 10f)
        {
            var turnadjust = (transform.eulerAngles.y - _oldRotation) * _steeerHelper;
            Quaternion velRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
            _car_Rigidbody.velocity = velRotation * _car_Rigidbody.velocity;
        }
        _oldRotation = transform.eulerAngles.y;
    }

    /// <summary>
    /// Расчет оборотов у машины
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
    /// Смена передач в зависимости от скорости
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
    /// Смена передач
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
    /// Создание ограничений скорости, мощности двигателя в зависимости от текущей передачи
    /// </summary>
    private void TorqueCurve()
    {
        //Максимальная скорость на текущую передачу
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
    /// Управление машиной
    /// </summary>
    /// <param name="steering">Horizontal управление</param>
    /// <param name="accel">Крутящий момент</param>
    /// <param name="footbrake">Vertical ножной тормоз</param>
    /// <param name="forceEngine">True если включен форсированый режим</param>
    public void Move(float steering, float accel, float footbrake, bool forceEngine)
    {
        //DebugInfo.Add("CurrentDirectionSpeed", (float)CurrentDirectionSpeed);

        // Ограничение входящих значений
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
    /// Остановить машину
    /// </summary>
    public void StopCar()
    {
        StartCoroutine(StopCarCoroutine());
    }

    /// <summary>
    /// Остановка машины
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
    /// Перерасчет всех характеристик в зависимости от левела компонента
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
    /// Применение х-к машины с сервера
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
    /// Нанесение урона машине
    /// </summary>
    /// <param name="damage">Урон</param>
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
    /// Реверсивная передача в настоящее время
    /// </summary>
    private int _direction = 1;
    /// <summary>
    /// Кол-во передач
    /// </summary>
    private int _totalGears = 8;
    /// <summary>
    /// Максимальная скорость
    /// </summary>
    private float _maxSpeed = 120;

    /// <summary>
    /// Угол поворота
    /// </summary>
    private float _maximimSteerAngle = 20;
    /// <summary>
    /// Сила торможения
    /// </summary>
    private float _brakeTorque = 2000f;
    /// <summary>
    /// Максимум оборотов двигателя
    /// </summary>
    private float _maxEngineRPM = 7000f;
    /// <summary>
    /// Минимальное кол-во оборотов двигателя
    /// </summary>
    private float _minEngineRPM = 1000f;
    [Range(0f, .5f)] private float _gearShiftingDelay = .35f;
    [Range(.5f, .95f)] private float _gearShiftingThreshold = .85f;
    [Range(.1f, .9f)] private float _clutchInertia = .25f;
    [Range(.75f, 2f)] private float _engineInertia = 1f;
    [Range(0f, 1f)] private float _TCSStrength = 1f;
    /// <summary>
    /// Коефициент при форсировании двигателя
    /// </summary>
    [Range(1f, 2f)] private float _forceEngine = 1.3f;
    /// <summary>
    /// 1 - Полная помощь на поворотах что бы машине не перевернулась
    /// </summary>
    [Range(0f, 1f)] private float _steeerHelper = 1f;

    private bool _useClutchMarginAtFirstGear = true;
    private float _idleInput = 0;
    private float _clutchInput = 0;
    private float _launched = 0;

    /// <summary>
    /// Коллайдеры колес
    /// </summary>
    [SerializeField]
    private List<WheelCollider> _wheelColliders = new List<WheelCollider>();
    /// <summary>
    /// Меши колес
    /// </summary>
    [SerializeField]
    private List<GameObject> _wheelMeshes = new List<GameObject>();
    /// <summary>
    /// Центр масы
    /// </summary>
    [SerializeField]
    private Transform _com;
    /// <summary>
    /// Rigidbody машины
    /// </summary>
    private Rigidbody _car_Rigidbody;
    /// <summary>
    /// Предыдущий угол поворота машины
    /// </summary>
    private float _oldRotation;
    /// <summary>
    /// True, если сейчас идет передключение коробки передач
    /// </summary>
    private bool _gearIsChanging = false;

    #endregion
}
