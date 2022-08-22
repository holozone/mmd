// Multiple motor driver
//

/* This file is part of mmd (multiple motor driver) from holozone.de.

    mmd is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Foobar is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details. 
*/

// Antrieb mehrerer Schrittmotoren über Pulsrichtung
//
//
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <limits.h>
#include <string.h>
#include <string>
#include <locale.h>
#include "mmd.h"
#include "drive.h"
#include "motordriver.h"
#include "file_parameter.h"

// Globals
// Menu status
enum class MODE {HAUPT} mode;
void print_menu();
bool ende;
// Target data
int target_n;
std::string target_desc;	
// behelfsweise Winkel des Probenmotors als Anzeigehilfe, hat nichts mit den Motoren hier zu tun
double motor_probe; 
// Time
struct timespec prog_global_time, prog_last_save_time, prog_startup_time;
double sec_up;
// GPIO and motor
gpio_driver_class gpio;
bool rt_thread_start();
bool rt_thread_stop();
pthread_t thread;
bool stop_thread;
bool local_settings_save();
bool local_settings_load();
bool local_position_save();
bool local_position_load();

// Mutex zur sichren Manipulation der datenstruktur von motordriver.h
extern pthread_mutex_t data_mutex;


//  
// Name, bm_puls, level_puls, bm_dir, level_dir, bm_sneg, l_sneg,  bm_spos, l_spos, bm_sref, l_sref, mm/step, vnorm, vmax, a_ramp, connected ,testmode
drive_data_class dd_main;
axis_data axis_a, axis_b, axis_c, axis_d, axis_e, axis_f;

int main() {
	// Zahlenformat vereinheitlichen
	setlocale(LC_NUMERIC, "C");
	// Zeit initialisieren
	clock_gettime(CLOCK_REALTIME, &prog_global_time);
	prog_startup_time = prog_global_time;
	sec_up = 0.0f;
	// Sicherheitshalber den gesamten Prozess locken
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                printf("mlockall failed: %m\n");
    }
	// menu 
	ende = false;
	dd_main.refresh = true;
	mode = MODE::HAUPT;
	char inchar[20];
	int inchar_n;
	canonmode(1);
	// Achsendaten laden
	// motors - insert here -
	// Pulse, Dir, sensor neg, Sensor plus
	// ------------------------------------
	// A 4,14,15,17
	// B 18,27,22,23
	// C 24,10,9,25
	// D 11,8,7,5
	// E 6,12,13,19
	// F 16,26,20,21
	// 
	// the bits and names are the same for every rack for now ...
	axis_a.init("3D1 x", 1<<4,   0, 1<<14,  1, 1<<15, 1, 1<<17,  1, 0, 0, 0.2/1600.0f, 0.5f, 1.0f, 1.0f, true, false);
	axis_b.init("3D1 y", 1<<18,  0, 1<<27,  1, 1<<22, 1, 1<<23,  1, 0, 0, 0.2/1600.0f, 1.0f, 2.0f, 1.0f, true, false);
	axis_c.init("3D1 z", 1<<24,  0, 1<<10,  1, 1<<9,  1, 1<<25,  1, 0, 0,   2/1600.0f, 0.5f, 1.0f, 1.0f, true, false);
	axis_d.init("3D2 x", 1<<11,  0, 1<<8,   1, 1<<7,  1, 1<<5,   1, 0, 0, 0.2/1600.0f, 0.5f, 1.0f, 1.0f, true, false);
	axis_e.init("3D2 y", 1<<6,   0, 1<<12,  1, 1<<13, 1, 1<<19,  1, 0, 0, 0.2/1600.0f, 0.5f, 1.0f, 1.0f, true, false);
	axis_f.init("3D2 z", 1<<16,  0, 1<<26,  1, 1<<20, 1, 1<<21,  1, 0, 0,   2/1600.0f, 0.5f, 1.0f, 1.0f, true, false);
	// Drive
	// setup gpio
	gpio.setup();
	// 3D1 x
	gpio.set_out(4); gpio.set_out(14); gpio.set_inp(15); gpio.set_inp(17);
	// 3D1 y
	gpio.set_out(18); gpio.set_out(27); gpio.set_inp(22); gpio.set_inp(23);
	// 3D1 z
	gpio.set_out(24); gpio.set_out(10); gpio.set_inp(9); gpio.set_inp(25);
	// 3D2 x
	gpio.set_out(11); gpio.set_out(8); gpio.set_inp(7); gpio.set_inp(5);
	// 3D2 y
	gpio.set_out(6); gpio.set_out(12); gpio.set_inp(13); gpio.set_inp(19);
	// 3D1 z
	gpio.set_out(16); gpio.set_out(26); gpio.set_inp(20); gpio.set_inp(21);
	// pointer to gpio
	dd_main.p_gpio = &gpio;
	// add axes
	dd_main.axis_array[0] = &axis_a;
	dd_main.axis_array[1] = &axis_b;
	dd_main.axis_array[2] = &axis_c;
	dd_main.axis_array[3] = &axis_d;
	dd_main.axis_array[4] = &axis_e;
	dd_main.axis_array[5] = &axis_f;
	dd_main.axis_end = 6;
	dd_main.axis_update();
	dd_main.refresh = true;
	// Daten + Positionen laden
	// Lokale Abweichungen laden 
	local_settings_load();
	// Letzte Position laden
	local_position_load();
	clock_gettime(CLOCK_REALTIME, &prog_last_save_time);
	// Mutex initialisieren 
	if (pthread_mutex_init(&data_mutex, NULL) != 0) printf("Creating Mutex failde \n");
	// rt thread starten
	rt_thread_start();
	// mainloop
	while(!ende) {
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Auf neue Eingabe prüfen
		inchar_n = 0;
		if (!keyhit()) {
			// Schlafen
			// printf("schnarch .... \n");
			clock_gettime(CLOCK_REALTIME, &prog_global_time);
			sec_up = diff_time(&prog_global_time, &prog_startup_time);
			// Alle 10 sekunden die aktuelle Position speichern
			if (diff_time(&prog_global_time, &prog_last_save_time) > 10.0) {
				local_position_save();
				clock_gettime(CLOCK_REALTIME, &prog_last_save_time); 
				dd_main.refresh = true;
			}
			// 50 ms schlafen
			usleep(50000);
		} else {
			// Auslesen eines oder mehrerer Zeichen
			inchar_n = 0;
			while (keyhit()) {
				read(0, &inchar[inchar_n], 1);
				// printf("Tastencode %d ist %d \n", inchar_n, inchar[inchar_n]);
				inchar_n++;
				if (inchar_n > 20) {
					printf("Fehler bei Lesen Tastatursequenz, Abbruch \n");
					break;
				}
			}
		}
		// Einzelzeichen auswerten
		if (inchar_n == 1) {
			switch (mode) {
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Hauptmenü
			// (Esc)Ende
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			case MODE::HAUPT:
				// Taste auswerten
				switch (inchar[0]) {
				// Achse wählen
				case '1':
					dd_main.p_active_axis = &axis_a;
					dd_main.refresh = true;
					break;
				case '2':
					dd_main.p_active_axis = &axis_b;
					dd_main.refresh = true;
					break;
				case '3':
					dd_main.p_active_axis = &axis_c;
					dd_main.refresh = true;
					break;
				case '4':
					dd_main.p_active_axis = &axis_d;
					dd_main.refresh = true;
					break;
				case '5':
					dd_main.p_active_axis = &axis_e;
					dd_main.refresh = true;
					break;
				case '6':
					dd_main.p_active_axis = &axis_f;
					dd_main.refresh = true;
					break;
				case '0':
					dd_main.p_active_axis = 0;
					dd_main.refresh = true;
					break;
				// Fahrbefehle
				case '+':
					// Kontinuierlich in Plus Richtung fahren
					if (dd_main.p_active_axis != 0) {
						dd_main.refresh = true;
						// counterclockwise/negativ
						if (!dd_main.motor_active_stop()) {
							// Bewegung in negativer Richtung
							dd_main.motor_active_gocont(POSITIV);
						}
					} else printf("Achse auswählen \n");
					break;
				case ' ':
					// Stoppen
					if (dd_main.p_active_axis != 0) {
						dd_main.refresh = true;
						dd_main.motor_active_stop();
					} else printf("Achse auswählen \n");	
					break;
				case '-':
					// Kontinuierlich in Minus Richtung fahren
					if (dd_main.p_active_axis != 0) {
						dd_main.refresh = true;
						// clockwise/positiv
						if (!dd_main.motor_active_stop()) {
							// Bewegung in positiver Richtung
							dd_main.motor_active_gocont(NEGATIV);
						}
					} else printf("Achse auswählen \n");	
					break;
				case 'd':
					// Description ändern
					if (dd_main.p_active_axis != 0) {
						dd_main.refresh = true;
						input = "";
							printf("Neue Beschreibung angeben: \n");
							while (collect_input(getchar()));
							printf("\n");
							if (input.length() > 0) {
								dd_main.p_active_axis->description = input;
							}
							dd_main.refresh = true;
					} else printf("Achse auswählen \n");					
					break;
				case 't':
					// Gespeichertes target anfahren
					if (dd_main.p_active_axis != 0) {
						dd_main.refresh = true;
						pthread_mutex_lock(&data_mutex);	
						dd_main.p_active_axis->v_fahr = dd_main.p_active_axis->v_std;
						dd_main.p_active_axis->init_ramp();
						dd_main.p_active_axis->go();
						pthread_mutex_unlock(&data_mutex);	
						dd_main.refresh = true;
					} else printf("Achse auswählen \n");					
					break;	
				case 'n':
					// Neues Target angeben
					if (dd_main.p_active_axis != 0) {
						dd_main.refresh = true;
						double pos_neu;
						input = "";
							printf("Neue Ziel-Position angeben \n");
							while (collect_input(getchar()));
							printf("\n");
							if (input.length() > 0) {
								pos_neu = strtof(input.c_str(), NULL);
								dd_main.p_active_axis->target = pos_neu;
							}
							dd_main.refresh = true;
					} else printf("Achse auswählen \n");					
					break;
				case 'r':
					// Referenz setzen
					if (dd_main.p_active_axis != 0) {
						double pos_neu;
						input = "";
						printf("Neue Position angeben: \n");
						while (collect_input(getchar()));
						if (input.length() > 0) {
							pos_neu = strtod(input.c_str(), NULL);
							dd_main.p_active_axis->set_ref_pos(pos_neu);
						}
						dd_main.refresh = true;
					} else printf("Achse auswählen \n");					
					break;
				case 'a':
					dd_main.motor_all_stop();
					dd_main.refresh = true;
					break;
				case 'l':
					// load targetdata from file mmd_targets.xml
					input = "";
					printf("Target Nr angeben \n");
					while (collect_input(getchar()));
					printf("\n");
					if (input.length() > 0) {
						target_n = atoi(input.c_str());
						double target_tmp;	
						if (get_file_parameter("mmd_targets.xml","target", target_n,"description", &target_desc))				
						if (get_file_parameter("mmd_targets.xml","target", target_n,"axis_a", &target_tmp))				
							axis_a.set_target(target_tmp);
						if (get_file_parameter("mmd_targets.xml","target", target_n,"axis_b", &target_tmp))				
							axis_b.set_target(target_tmp);
						if (get_file_parameter("mmd_targets.xml","target", target_n,"axis_c", &target_tmp))				
							axis_c.set_target(target_tmp);
						if (get_file_parameter("mmd_targets.xml","target", target_n,"axis_d", &target_tmp))				
							axis_d.set_target(target_tmp);
						if (get_file_parameter("mmd_targets.xml","target", target_n,"axis_e", &target_tmp))				
							axis_e.set_target(target_tmp);
						if (get_file_parameter("mmd_targets.xml","target", target_n,"axis_f", &target_tmp))				
							axis_f.set_target(target_tmp);
						if (get_file_parameter("mmd_targets.xml","target", target_n,"motor_probe", &target_tmp))				
							motor_probe = target_tmp;
					}
					dd_main.refresh = true;
					break;
				case 'x':
					// Alle neu geladenen targets anfahren (siehe load)
					axis_a.go_target();
					axis_b.go_target();
					axis_c.go_target();
					axis_d.go_target();
					axis_e.go_target();
					axis_f.go_target();
					break;
				// Anderes			
				case 27:
					// Escape -> Motor Stopp oder Programmende
					printf("Escape, Abbruch \n");
					ende = true;
					break;
				}
			}
		}
		// Menu
		if (dd_main.refresh) print_menu(), dd_main.refresh = false;
	}
	// Ende
	rt_thread_stop();
	pthread_mutex_destroy(&data_mutex);
	// Daten + Positionen speichern
	local_position_save();
	// Back ...
	canonmode(0);
}

void print_menu() { 
	printf("\n\n\n\n");
	printf("============================================================================== \n");
	printf("mmd multiple motor driver 		uptime: %.1lf\n", sec_up);
	printf(" 								\n");
	printf("1x:%3.2lf %d %d t%3.2lf \t1y:%3.2lf %d %d t%3.2lf \t1z:%3.2lf %d %d t%3.2lf \n", \
									 	axis_a.get_pos(), axis_a.sneg_on, axis_a.spos_on, axis_a.target, \
										axis_b.get_pos(), axis_b.sneg_on, axis_b.spos_on, axis_b.target, \
										axis_c.get_pos(), axis_c.sneg_on, axis_c.spos_on, axis_c.target );
	printf(" 								  \n");
	printf("1x:%3.2lf %d %d t%3.2lf \t1y:%3.2lf %d %d t%3.2lf \t1z:%3.2lf %d %d t%3.2lf \n", \
	 									axis_d.get_pos(), axis_d.sneg_on, axis_d.spos_on, axis_d.target, \
										axis_e.get_pos(), axis_e.sneg_on, axis_e.spos_on, axis_e.target, \
										axis_f.get_pos(), axis_f.sneg_on, axis_f.spos_on, axis_f.target );
	printf(" 								  \n");
	if (dd_main.motor_active()) {
		printf(" Aktive Achse %s			 \n", dd_main.motor_active_name().c_str());
		printf(" Description %s			         \n", dd_main.motor_active_description().c_str());
		printf(" Mode %s			         \n", dd_main.motor_active_mode());
	}
	printf(" 								  \n");
	printf("============================================================================ \n");
	printf(" Achsenauswahl 1,2,3,4,5,6 0 = keine                     \n");
	printf(" Fahren (+) (-)  (SPACE) = Stop  	                 \n");
	printf(" (t)arget anfahren (n)eues target setzen (r)eferenzposition neu (d)escription ändern \n");
	printf(" (l)oad targetdata from file e(x)ecute all  (a)lle Achsen Stop \n");
	printf(" 					 \n");
	printf(" Aktuelles target: %s , Nr. %d                              \n", target_desc.c_str(), target_n );
	// printf(" Motor Probe Winkel %f                                   \n", motor_probe );
	printf(" 							 \n");
	printf(" (ESC) Ende\n");
}

// Motorpositionen laden und speichern
bool local_settings_save() {
	// Save the config-file with fprintf
	FILE* fd_file;
	fd_file = fopen("mmd_local_config.xml","w+");	// Auch ÜBERschreiben
	if (fd_file != NULL) {
		fprintf(fd_file, "<text> XML-konfigurationsfile mmd </text> \n");
		// Konfigurationsdaten speichern
		// Achsen-Beschreibung speichern
		fprintf(fd_file, "<axis_a desc=\"%s\"/> \n", axis_a.description.c_str());
		fprintf(fd_file, "<axis_b desc=\"%s\"/> \n", axis_b.description.c_str());
		fprintf(fd_file, "<axis_c desc=\"%s\"/> \n", axis_c.description.c_str());
		fprintf(fd_file, "<axis_d desc=\"%s\"/> \n", axis_d.description.c_str());
		fprintf(fd_file, "<axis_e desc=\"%s\"/> \n", axis_e.description.c_str());
		fprintf(fd_file, "<axis_f desc=\"%s\"/> \n", axis_f.description.c_str());
		// Ende
		fclose(fd_file);
		return true;
	} else {
		printf("Konnte Konfigurations-Datei nicht speichern \n");
		return false;
	}
}

bool local_settings_load() {
	// Achsen-Namen laden
	get_file_parameter("mmd_local_config.xml","axis_a",1,"name", &axis_a.name);
	get_file_parameter("mmd_local_config.xml","axis_b",1,"name", &axis_b.name);
	get_file_parameter("mmd_local_config.xml","axis_c",1,"name", &axis_c.name);
	get_file_parameter("mmd_local_config.xml","axis_d",1,"name", &axis_d.name);
	get_file_parameter("mmd_local_config.xml","axis_e",1,"name", &axis_e.name);
	get_file_parameter("mmd_local_config.xml","axis_f",1,"name", &axis_f.name);
	// Achsen-pds laden (Position in Zieleinheit / Steps) 
	get_file_parameter("mmd_local_config.xml","axis_a",1,"pds", &axis_a.pos_div_step);
	get_file_parameter("mmd_local_config.xml","axis_b",1,"pds", &axis_b.pos_div_step);
	get_file_parameter("mmd_local_config.xml","axis_c",1,"pds", &axis_c.pos_div_step);
	get_file_parameter("mmd_local_config.xml","axis_d",1,"pds", &axis_d.pos_div_step);
	get_file_parameter("mmd_local_config.xml","axis_e",1,"pds", &axis_e.pos_div_step);
	get_file_parameter("mmd_local_config.xml","axis_f",1,"pds", &axis_f.pos_div_step);
	// Achsen-Beschreibungen laden
	get_file_parameter("mmd_local_config.xml","axis_a",1,"desc", &axis_a.description);
	get_file_parameter("mmd_local_config.xml","axis_b",1,"desc", &axis_b.description);
	get_file_parameter("mmd_local_config.xml","axis_c",1,"desc", &axis_c.description);
	get_file_parameter("mmd_local_config.xml","axis_d",1,"desc", &axis_d.description);
	get_file_parameter("mmd_local_config.xml","axis_e",1,"desc", &axis_e.description);
	get_file_parameter("mmd_local_config.xml","axis_f",1,"desc", &axis_f.description);
	return true;
}

// Aktuelle Position zwischenspeichern (zu Programmenden und sicherheitshalber, falls Stromunterbrechung etc.)
bool local_position_save() {
	// Save the config-file with fprintf
	FILE* fd_file;
	fd_file = fopen("mmd_local_position.xml","w+");	// Auch ÜBERschreiben
	if (fd_file != NULL) {
		fprintf(fd_file, "<text> XML-Sicherung der mmd Positionsdaten </text> \n");
		// Motorpositionen speichern
		fprintf(fd_file, "<axis_a pos=\"%lf\" /> \n", axis_a.get_pos());
		fprintf(fd_file, "<axis_b pos=\"%lf\" /> \n", axis_b.get_pos());
		fprintf(fd_file, "<axis_c pos=\"%lf\" /> \n", axis_c.get_pos());
		fprintf(fd_file, "<axis_d pos=\"%lf\" /> \n", axis_d.get_pos());
		fprintf(fd_file, "<axis_e pos=\"%lf\" /> \n", axis_e.get_pos());
		fprintf(fd_file, "<axis_f pos=\"%lf\" /> \n", axis_f.get_pos());
		// Ende
		fclose(fd_file);
		return true;
	} else {
		printf("Konnte Konfigurations-Datei nicht speichern \n");
		return false;
	}
}

bool local_position_load() {
	// Motorpositionen laden
	axis_a.set_ref_pos(get_parameter("mmd_local_position.xml","axis_a",1,"pos", 0.0f));
	axis_b.set_ref_pos(get_parameter("mmd_local_position.xml","axis_b",1,"pos", 0.0f));
	axis_c.set_ref_pos(get_parameter("mmd_local_position.xml","axis_c",1,"pos", 0.0f));
	axis_d.set_ref_pos(get_parameter("mmd_local_position.xml","axis_d",1,"pos", 0.0f));
	axis_e.set_ref_pos(get_parameter("mmd_local_position.xml","axis_e",1,"pos", 0.0f));
	axis_f.set_ref_pos(get_parameter("mmd_local_position.xml","axis_f",1,"pos", 0.0f));
	return true;
}


void ts_time_add(struct timespec* p_ts_op, long ns2add) {
	if (p_ts_op->tv_nsec + ns2add < 1000000000) p_ts_op->tv_nsec += ns2add; else p_ts_op->tv_sec += 1, p_ts_op->tv_nsec = p_ts_op->tv_nsec + ns2add - 1000000000;
}

long diff_ts(struct timespec* p_ts_op1, struct timespec* p_ts_op2) {
	return (p_ts_op1->tv_nsec - p_ts_op2->tv_nsec) + (p_ts_op1->tv_sec - p_ts_op2->tv_sec)*1000000000;
}


bool rt_thread_start() {
	int ret;
	struct sched_param param;
        pthread_attr_t attr;
        // printf("maximal prio is %d  \n", sched_get_priority_max(SCHED_FIFO));
        dd_main.run = true;
        // Lock memory
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                printf("mlockall failed: %m\n");
                goto out;
        }
        // Initialize pthread attributes (default values) 
        ret = pthread_attr_init(&attr);
        if (ret) {
                printf("init pthread attributes failed\n");
                goto out;
        }
        // Set a specific stack size  
        ret = pthread_attr_setstacksize(&attr, 255 * 4096 + PTHREAD_STACK_MIN);
        if (ret) {
            printf("pthread setstacksize failed\n");
            goto out;
        }
        // Set scheduler policy and priority of pthread 
        ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (ret) {
                printf("pthread setschedpolicy failed\n");
                goto out;
        }
        param.sched_priority = 99;
        ret = pthread_attr_setschedparam(&attr, &param);
        if (ret) {
                printf("pthread setschedparam failed\n");
                goto out;
        }
        // Use scheduling parameters of attr 
        ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (ret) {
                printf("pthread setinheritsched failed\n");
                goto out;
        }
        // Create a pthread with specified attributes 
        /*
        ret = pthread_create(&thread, &attr, thread_func, NULL);
        if (ret) {
                printf("create pthread failed\n");
                goto out;
        }
        */
        // 
        ret = pthread_create(&thread, &attr, &motor_ramp_driver, &dd_main);
        if (ret) {
                printf("create pthread failed\n");
                goto out;
        }
        // Everything done
        return true;
 out:
       return true;
}

bool rt_thread_stop() {
	int ret;
	// Stop it
	// stop_thread = true;
	dd_main.run = false;
    	// Join the thread and wait until it is done 
	ret = pthread_join(thread, NULL);
    	if (ret) printf("join pthread failed: %m\n");
    	return true;
}

