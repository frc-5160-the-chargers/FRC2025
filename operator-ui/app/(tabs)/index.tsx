import {StyleSheet, Text, View} from "react-native";
import {useEffect} from "react";
import * as ScreenOrientation from 'expo-screen-orientation'
import {OrientationLock} from 'expo-screen-orientation'
import {useLoadNtRequest} from "@/ext/NetworkTablesState";
import {PathfindTargetSelector} from "@/components/PathfindTargetSelector";
import {AllLevelsSelector} from "@/components/AllLevelsSelector";

export default function Index() {
    const loadRequest = useLoadNtRequest()
    useEffect(() => {
        ScreenOrientation.lockAsync(OrientationLock.LANDSCAPE).then()
    }, [])

    let loadingMsg = ""
    if (loadRequest.isSuccess) {
        loadingMsg = "Success!"
    } else if (loadRequest.isError) {
        loadingMsg = "Error: " + loadRequest.error
    } else {
        loadingMsg = "Loading...(requests will fail atm)"
    }

    console.log("I should be here....")

    return (
        <View style={styles.container}>
            <Text style={{width: 100, marginLeft: 0}}>{loadingMsg}</Text>
            <PathfindTargetSelector />
            <AllLevelsSelector />
        </View>
    );
}

const styles = StyleSheet.create({
    container: {
        paddingHorizontal: 40,
        paddingVertical: 20,
        flexDirection: "row",
        gap: 0
    },
    hexagonImage: {
        width: 200,
        height: 200,
        marginLeft: 200,
        marginTop: 40
    },
    Button: {
        backgroundColor: "rgb(106,106,215)"
    }
})
