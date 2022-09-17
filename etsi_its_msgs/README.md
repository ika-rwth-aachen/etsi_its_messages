### CAM

<details>
<summary>Show message</summary>

```yml
header: 
  protocolVersion: 0
  messageID: 0
  stationID: 
    value: 0
cam: 
  generationDeltaTime: 
    value: 0
  camParameters: 
    basicContainer: 
      stationType: 
        value: 0
      referencePosition: 
        latitude: 
          value: 0
        longitude: 
          value: 0
        positionConfidenceEllipse: 
          semiMajorConfidence: 
            value: 0
          semiMinorConfidence: 
            value: 0
          semiMajorOrientation: 
            value: 0
        altitude: 
          altitudeValue: 
            value: 0
          altitudeConfidence: 
            value: 0
    highFrequencyContainer: 
      choice: 0
      basicVehicleContainerHighFrequency: 
        heading: 
          headingValue: 
            value: 0
          headingConfidence: 
            value: 0
        speed: 
          speedValue: 
            value: 0
          speedConfidence: 
            value: 0
        driveDirection: 
          value: 0
        vehicleLength: 
          vehicleLengthValue: 
            value: 0
          vehicleLengthConfidenceIndication: 
            value: 0
        vehicleWidth: 
          value: 0
        longitudinalAcceleration: 
          longitudinalAccelerationValue: 
            value: 0
          longitudinalAccelerationConfidence: 
            value: 0
        curvature: 
          curvatureValue: 
            value: 0
          curvatureConfidence: 
            value: 0
        curvatureCalculationMode: 
          value: 0
        yawRate: 
          yawRateValue: 
            value: 0
          yawRateConfidence: 
            value: 0
        accelerationControl: 
          value: ''
        lanePosition: 
          value: 0
        steeringWheelAngle: 
          steeringWheelAngleValue: 
            value: 0
          steeringWheelAngleConfidence: 
            value: 0
        lateralAcceleration: 
          lateralAccelerationValue: 
            value: 0
          lateralAccelerationConfidence: 
            value: 0
        verticalAcceleration: 
          verticalAccelerationValue: 
            value: 0
          verticalAccelerationConfidence: 
            value: 0
        performanceClass: 
          value: 0
        cenDsrcTollingZone: 
          protectedZoneLatitude: 
            value: 0
          protectedZoneLongitude: 
            value: 0
          cenDsrcTollingZoneID: 
            value: 
              value: 0
      rsuContainerHighFrequency: 
        protectedCommunicationZonesRSU: 
          array: []
    lowFrequencyContainer: 
      choice: 0
      basicVehicleContainerLowFrequency: 
        vehicleRole: 
          value: 0
        exteriorLights: 
          value: ''
        pathHistory: 
          array: []
    specialVehicleContainer: 
      choice: 0
      publicTransportContainer: 
        embarkationStatus: 
          value: False
        ptActivation: 
          ptActivationType: 
            value: 0
          ptActivationData: 
            value: ''
      specialTransportContainer: 
        specialTransportType: 
          value: ''
        lightBarSirenInUse: 
          value: ''
      dangerousGoodsContainer: 
        dangerousGoodsBasic: 
          value: 0
      roadWorksContainerBasic: 
        roadworksSubCauseCode: 
          value: 0
        lightBarSirenInUse: 
          value: ''
        closedLanes: 
          innerhardShoulderStatus: 
            value: 0
          outerhardShoulderStatus: 
            value: 0
          drivingLaneStatus: 
            value: ''
      rescueContainer: 
        lightBarSirenInUse: 
          value: ''
      emergencyContainer: 
        lightBarSirenInUse: 
          value: ''
        incidentIndication: 
          causeCode: 
            value: 0
          subCauseCode: 
            value: 0
        emergencyPriority: 
          value: ''
      safetyCarContainer: 
        lightBarSirenInUse: 
          value: ''
        incidentIndication: 
          causeCode: 
            value: 0
          subCauseCode: 
            value: 0
        trafficRule: 
          value: 0
        speedLimit: 
          value: 0
```

</details>


### DENM

<details>
<summary>Show message</summary>

```yml
header: 
  protocolVersion: 0
  messageID: 0
  stationID: 
    value: 0
denm: 
  management: 
    actionID: 
      originatingStationID: 
        value: 0
      sequenceNumber: 
        value: 0
    detectionTime: 
      value: 0
    referenceTime: 
      value: 0
    termination: 
      value: 0
    eventPosition: 
      latitude: 
        value: 0
      longitude: 
        value: 0
      positionConfidenceEllipse: 
        semiMajorConfidence: 
          value: 0
        semiMinorConfidence: 
          value: 0
        semiMajorOrientation: 
          value: 0
      altitude: 
        altitudeValue: 
          value: 0
        altitudeConfidence: 
          value: 0
    relevanceDistance: 
      value: 0
    relevanceTrafficDirection: 
      value: 0
    validityDuration: 
      value: 0
    transmissionInterval: 
      value: 0
    stationType: 
      value: 0
  situation: 
    informationQuality: 
      value: 0
    eventType: 
      causeCode: 
        value: 0
      subCauseCode: 
        value: 0
    linkedCause: 
      causeCode: 
        value: 0
      subCauseCode: 
        value: 0
    eventHistory: 
      array: []
  location: 
    eventSpeed: 
      speedValue: 
        value: 0
      speedConfidence: 
        value: 0
    eventPositionHeading: 
      headingValue: 
        value: 0
      headingConfidence: 
        value: 0
    traces: 
      array: []
    roadType: 
      value: 0
  alacarte: 
    lanePosition: 
      value: 0
    impactReduction: 
      heightLonCarrLeft: 
        value: 0
      heightLonCarrRight: 
        value: 0
      posLonCarrLeft: 
        value: 0
      posLonCarrRight: 
        value: 0
      positionOfPillars: 
        array: []
      posCentMass: 
        value: 0
      wheelBaseVehicle: 
        value: 0
      turningRadius: 
        value: 0
      posFrontAx: 
        value: 0
      positionOfOccupants: 
        value: ''
      vehicleMass: 
        value: 0
      requestResponseIndication: 
        value: 0
    externalTemperature: 
      value: 0
    roadWorks: 
      lightBarSirenInUse: 
        value: ''
      closedLanes: 
        innerhardShoulderStatus: 
          value: 0
        outerhardShoulderStatus: 
          value: 0
        drivingLaneStatus: 
          value: ''
      restriction: 
        array: []
      speedLimit: 
        value: 0
      incidentIndication: 
        causeCode: 
          value: 0
        subCauseCode: 
          value: 0
      recommendedPath: 
        array: []
      startingPointSpeedLimit: 
        deltaLatitude: 
          value: 0
        deltaLongitude: 
          value: 0
        deltaAltitude: 
          value: 0
      trafficFlowRule: 
        value: 0
      referenceDenms: 
        array: []
    positioningSolution: 
      value: 0
    stationaryVehicle: 
      stationarySince: 
        value: 0
      stationaryCause: 
        causeCode: 
          value: 0
        subCauseCode: 
          value: 0
      carryingDangerousGoods: 
        dangerousGoodsType: 
          value: 0
        unNumber: 0
        elevatedTemperature: False
        tunnelsRestricted: False
        limitedQuantity: False
        emergencyActionCode: ''
        phoneNumber: 
          value: ''
        companyName: ''
      numberOfOccupants: 
        value: 0
      vehicleIdentification: 
        wMInumber: 
          value: ''
        vDS: 
          value: ''
      energyStorageType: 
        value: ''
```

</details>
