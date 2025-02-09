import {StrictMode} from 'react'
import {createRoot} from 'react-dom/client'
import '@mantine/core/styles.css';
import {App} from './App.tsx'
import {MantineProvider} from "@mantine/core";
import {THEME} from "./theme.ts";

createRoot(document.getElementById('root')!).render(
    <MantineProvider theme={THEME} >
        <StrictMode>
            <App/>
        </StrictMode>
    </MantineProvider>,
)
