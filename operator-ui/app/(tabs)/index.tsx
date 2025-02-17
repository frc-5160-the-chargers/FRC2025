import {Button, StyleSheet, Text, View} from "react-native";
import {useEffect, useState} from "react";
import {useNtPublisher} from "@/app/nt4/useNtPublisher";
import {NT4_Client} from "@/app/nt4/NT4";

const IS_SIM = false

export function Index() {
    const [isLoading, setLoading] = useState(false)
    const [client] = useState(
        new NT4_Client(
            IS_SIM ? "127.0.0.1" : "10.51.60.2",
            "OperatorUI",
            () => {},
            () => {},
            () => {},
            () => setLoading(false),
            () => setLoading(true)
        )
    )

    const [, setRawValue] = useNtPublisher(client, "test/test2", "boolean", false)

    useEffect(() => {
        client.connect().then(() => console.log("Client connected"))
        addEventListener("pagehide", () => client.disconnect())
    }, []);

    return (
        <View style={styles}>
            <Text>{isLoading ? "Loading...." : "Loaded!"}</Text>
            <Button
                title="Broadcast test"
                disabled={isLoading}
                onPress={() => setRawValue(true)}
            />
            <Text>Edit app/index.tsx to edit this screen.</Text>
        </View>
    );
}

const styles = StyleSheet.create({

})
