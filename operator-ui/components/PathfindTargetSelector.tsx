import {Image, StyleSheet, Text, TouchableOpacity, View, ViewStyle} from "react-native";
import {useNtState} from "@/ext/NetworkTablesState";

export function PathfindTargetSelector() {
    return (
        <View>
            <View style={styles.topBottomRows}>
                <PathfindSelectComponent pathfindTarget={6} />
                <PathfindSelectComponent pathfindTarget={7} />
            </View>
            <View style={styles.midRows}>
                <PathfindSelectComponent pathfindTarget={5} />
                <PathfindSelectComponent pathfindTarget={8} />
            </View>
            <View style={styles.centerRow}>
                <PathfindSelectComponent pathfindTarget={4} />
                <PathfindSelectComponent pathfindTarget={9} />
            </View>
            <View style={styles.centerRow}>
                <PathfindSelectComponent pathfindTarget={3} />
                <PathfindSelectComponent pathfindTarget={10} />
            </View>
            <View style={styles.midRows}>
                <PathfindSelectComponent pathfindTarget={2} />
                <PathfindSelectComponent pathfindTarget={11} />
            </View>
            <View style={styles.topBottomRows}>
                <PathfindSelectComponent pathfindTarget={1} />
                <PathfindSelectComponent pathfindTarget={0} />
            </View>
        </View>
    )
}

/**
 * IMPORTANT: ID must be distinct for every button
 */
function PathfindSelectComponent(args: {pathfindTarget: number, style?: ViewStyle}) {
    const currentTarget = useNtState(state => state.pathfindTarget)
    const setTarget = useNtState(state => state.setPathfindTarget)
    return <TouchableOpacity
        style={StyleSheet.compose(styles.button, args.style)}
        onPress={() => setTarget(args.pathfindTarget)}
    />
}

const styles = StyleSheet.create({
    button: {
        width: 50,
        height: 50,
        borderRadius: 10,
        backgroundColor: "blue"
    },
    topBottomRows: {
        flexDirection: "row", 
        gap: 30,
        marginLeft: 110,
        marginBottom: 10
    },
    midRows: {
        flexDirection: "row",
        gap: 120,
        marginLeft: 65,
        marginBottom: 10
    },
    centerRow: {
        flexDirection: "row",
        gap: 210,
        marginLeft: 20,
        marginBottom: 10
    }
})
