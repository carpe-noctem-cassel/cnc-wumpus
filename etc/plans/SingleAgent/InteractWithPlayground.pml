<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1534836538908" name="InteractWithPlayground" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1534836538909" name="Wait" comment="" entryPoint="1534836538910">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/WaitForTurn.beh#1534835374750</plans>
    <inTransitions>#1534836627481</inTransitions>
    <outTransitions>#1534836594030</outTransitions>
  </states>
  <states id="1534836578485" name="MyTurn" comment="">
    <plans xsi:type="alica:Plan">MyTurn.pml#1534836685239</plans>
    <inTransitions>#1534836594030</inTransitions>
    <outTransitions>#1534836627481</outTransitions>
  </states>
  <transitions id="1534836594030" name="MISSING_NAME" comment="wait4turn success" msg="">
    <preCondition id="1534836595527" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534836538909</inState>
    <outState>#1534836578485</outState>
  </transitions>
  <transitions id="1534836627481" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1534836628585" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1534836578485</inState>
    <outState>#1534836538909</outState>
  </transitions>
  <entryPoints id="1534836538910" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1414681164704</task>
    <state>#1534836538909</state>
  </entryPoints>
</alica:Plan>
