package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.Superstructure;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class OperatorTab extends ShuffleboardTabBase {

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Operator");
    }

    @Override
    public void update() {
    }

}
