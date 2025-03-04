import {create} from "zustand/react";
import {NetworkTables, NetworkTablesTypeInfos} from "ntcore-ts-client";
import {useQuery} from "@tanstack/react-query";
import {delay} from "@/ext/delay";
import {useEffect} from "react";
import {combine} from "zustand/middleware";

enum Mode {
    REAL_ROBOT, DANIEL_IMAC_SIM, DRIVER_STATION_SIM
}

function getNtClient(mode: Mode) {
    switch (mode) {
        case Mode.REAL_ROBOT: return NetworkTables.getInstanceByTeam(5160)
        case Mode.DANIEL_IMAC_SIM: return NetworkTables.getInstanceByURI("192.168.68.106")
        case Mode.DRIVER_STATION_SIM: return NetworkTables.getInstanceByURI("TBD") // TODO
    }
}

const waitPeriodMs = 100
const ntClient = getNtClient(Mode.DANIEL_IMAC_SIM)
const pathfindTargetTopic = ntClient.createTopic<number>(
    "operatorUi/pathfindTarget",
    NetworkTablesTypeInfos.kDouble
)
const targetLevelTopic = ntClient.createTopic<number>(
    "operatorUi/targetLevel",
    NetworkTablesTypeInfos.kDouble
)
const manualOverrideTopic = ntClient.createTopic<boolean>(
    "operatorUi/manualOverrideEnabled",
    NetworkTablesTypeInfos.kBoolean
)

/**
 * A query that ensures that networktables is loaded, and tracks the state
 * of the loading process.
 */
export function useLoadNtRequest() {
    const setReady = useNtState(state => state.setReady)
    const loadRequest = useQuery({
        queryKey: ["preloadRequest"],
        queryFn: async () => {
            while (!ntClient.isRobotConnected()) {
                await delay(waitPeriodMs)
            }
            await Promise.all([
                pathfindTargetTopic.publish(),
                targetLevelTopic.publish(),
                manualOverrideTopic.publish()
            ])
            setReady(true)
            return true
        },
        staleTime: Infinity
    })
    useEffect(() => {
        if (loadRequest.isSuccess && !ntClient.isRobotConnected()) {
            loadRequest.refetch().then()
        }
    }, [loadRequest]);
    return loadRequest;
}


export const useNtState = create(
    combine(
        {
            ready: false,
            pathfindTarget: -1,
            targetLevel: -1,
            manualOverrideEnabled: false,
        },
        (set, get) => ({
            setReady: (value: boolean) => set({ready: value}),
            setPathfindTarget: (target: number) => {
                if (!get().ready) return
                pathfindTargetTopic.setValue(target)
                console.log("Pathfind Target being set: " + target)
                set({pathfindTarget: target})
            },
            setTargetLevel: (target: number) => {
                if (!get().ready) return
                targetLevelTopic.setValue(target)
                console.log("Target level being set: " + target)
                set({targetLevel: target})
            },
            enableManualOverride: () => {
                if (!get().ready) return
                manualOverrideTopic.setValue(true)
                set({manualOverrideEnabled: true})
            },
            disableManualOverride: () => {
                if (!get().ready) return
                manualOverrideTopic.setValue(false)
                set({manualOverrideEnabled: false})
            }
        })
    )
)