import {useNtState} from "@/ext/NetworkTablesState";
import {StyleSheet, Text, TouchableOpacity, View} from "react-native";

export function AllLevelsSelector() {
    const currentLevel = useNtState(state => state.targetLevel)
    const setCurrentLevel = useNtState(state => state.setTargetLevel)
    return (
        <View style={{marginLeft: 70}}>
            {
                [4, 3, 2, 1].map(level =>
                    <TouchableOpacity
                        style={StyleSheet.compose(styles.button, currentLevel === level ? styles.active : styles.inactive)}
                        onPress={() => setCurrentLevel(level)}
                    >
                        <Text style={{color: "white"}}>L{level}</Text>
                    </TouchableOpacity>
                )
            }
        </View>
    )
}

const styles = StyleSheet.create({
    button: {
        margin: 20,
        borderRadius: 10,
        paddingHorizontal: 40,
        paddingVertical: 18,
        backgroundColor: "red",
        marginBottom: 15
    },
    inactive: {
        backgroundColor: "red"
    },
    active: {
        backgroundColor: "orange"
    }
})