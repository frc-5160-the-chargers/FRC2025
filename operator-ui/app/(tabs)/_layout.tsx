import {Tabs} from "expo-router";

export function TabsLayout() {
    return (
        <Tabs>
            <Tabs.Screen name="index" options={{title: "Home"}} />
            <Tabs.Screen name="guide" options={{title: "Guide"}} />
        </Tabs>
    )
}