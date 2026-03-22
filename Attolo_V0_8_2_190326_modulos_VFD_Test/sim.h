#pragma once

extern bool g_sim_di[32];
extern bool g_sim_do[32];

void simSetup();
void simLoop();

void simHelp();
void simShow();