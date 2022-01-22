package org.usfirst.frc3707.Creedence.Configuration;

public class TunableParameters {

    /**
     * Gets the value for the tunable parameter: LidarEnabled
     * 
     * @param allowUseNetworkTables Allow the use of NetworkTables, if the data is
     *                              available
     * @param allowUseLocalStorage  Allow the use of local storage, if the data is
     *                              available
     */
    public static boolean getLidarEnabled(boolean allowUseNetworkTables, boolean allowUseLocalStorage) {
        // if data is available in NetworkTables, and is set to use it, get it from
        // there

        // if data is available in local storage, and is set to use it, get it from
        // there

        // else, fallback to the default option.
        return false;
    }
}