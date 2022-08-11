package com.team1678.frc2022.shuffleboard;

import java.util.ArrayList;
import java.util.List;

import com.team1678.frc2022.shuffleboard.tabs.OperatorTab;
import com.team1678.frc2022.shuffleboard.tabs.SwerveTab;
import com.team1678.frc2022.shuffleboard.tabs.VisionTab;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleBoardInteractions {

    // Trims unneccesary tabs when in competition
    public final boolean mDebug = false;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    private OperatorTab mOperatorTab;
    private FieldView mFieldView = new FieldView();

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        mOperatorTab = new OperatorTab();
        mTabs.add(mOperatorTab);

        if (mDebug) {
            List<ShuffleboardTabBase> optionalTabs = List.of(
                new SwerveTab(),
                new VisionTab()
            );
            mTabs.addAll(optionalTabs);
        }

        for(ShuffleboardTabBase tab: mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
        mFieldView.update();
    }

    public ShuffleboardTab getOperatorTab() {
        return mOperatorTab.getTab();
    }
}
 