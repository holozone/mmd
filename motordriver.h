// motordriver.h
//
// mehrere Motorachsen gleichzeitig Fahren
#include <string>
#include <math.h>
#include <pthread.h>


pthread_mutex_t data_mutex; 

double diff_time(struct timespec* pts1, struct timespec* pts2);
void ts_add_double(struct timespec* p_to_add, double ns2add);
enum direction {POSITIV, NEGATIV};
long debug = 0;

class axis_data {
public:
	// Constructors
	axis_data() {
		set();
	}
	axis_data(std::string n2s) {
		set();
		name = n2s;
	}
	// Name, bm_puls, level_puls, bm_dir, level_dir, bm_sneg, l_sneg,  bm_spos, l_spos, bm_sref, l_sref, mm/step, vnorm, vmax, a_ramp, connected ,testmode?
	axis_data(std::string n2s, unsigned int bmp1, int lpp, unsigned int bmd1, int lpd,  unsigned int bm_sneg, int l_sneg,\
	          unsigned int bm_spos, int l_spos, unsigned int bm_sref, int l_sref, double pds1, double vstd1, double vmax1, double astd1, bool con1, bool test1) {
		set();
		init(n2s, bmp1, lpp, bmd1, lpd, bm_sneg, l_sneg, bm_spos, l_spos, bm_sref, l_sref, pds1, vstd1, vmax1, astd1, con1, test1);
	}
public:
	// Preset
	void set() {
		running = false;
		// dir_positiv = true;
		// pos = 0;
		connected = false;							// Achse aus
		offset = 0;									// Offset in mm
		pos_div_step = 1.0f / 200.0f;				// mm oder Grad pro step
		steps = 0;									// Absolute Schrittzahl
		bitmask_pulse = 0;							// bit für pulsausgang
		bitmask_dir = 0;							// bit für richtungsausgang
		level_pos_pulse = 0;						// wird nicht ausgwertet
		level_pos_dir = 0;							// wird benutzt
		v_min = 0.1f;								// Start/Mindestgeschwindigkeit in grad/mm pro s
		v_fahr = v_soll = v_std;					// Arbeitsgeschwindigkeit = Standard in grad/mm pro s
		v_max = 30.0f;								// Maximalgeschwindigkeit in grad/mm pro s
		pulse_length = 0.00005;						// 50 µs Pulslänge
		pulse_last = -1;							// als "ungültig" markieren
		pulse_next = -1;							// als "ungültig" markieren
	}
	// Init
	bool init(std::string n2s, unsigned int bmp1, int lpp, unsigned int bmd1, int lpd,  unsigned int bm_sneg, int l_sneg,\
	          unsigned int bm_spos, int l_spos, unsigned int bm_sref, int l_sref, double pds1, double vstd1, double vmax1, double astd1, bool con1, bool test1) {
		name = n2s;
		connected = con1;
		pos_div_step = pds1;
		bitmask_pulse = bmp1;
		bitmask_dir = bmd1;
		level_pos_pulse = lpp;
		level_pos_dir = lpd;
		test = test1;
		v_fahr = v_std = vstd1;
		v_max = vmax1;
		a_std = astd1;
		// Datenübernahme Schalter fehlt!
		bitmask_spos = bm_spos;
		level_on_spos = l_spos;
		bitmask_sneg = bm_sneg;
		level_on_sneg = l_sneg;
		bitmask_sref = bm_sref;
		level_on_sref = l_sref;
		return true;
	}
	// Name des aktuellen zustandes
	const char* modus_str() {
		return rstat_namen[rs];
	}
	// Get the actual position in mm / Grad
	// Setzt last_pos und pos neu!
	double get_pos() {
		// remember pos as last_pos and recalc
		last_pos = pos;
		pos =  pos_div_step * (double) steps + offset;
		return pos;
	}
	double peek_pos() {
		// .. without changing pos
		return pos_div_step * (double) steps + offset;
	}
	// Set the offset so that the current position match the given value
	bool set_ref_pos(double new_pos) {
		offset = new_pos - pos_div_step * (double) steps;
		return true;
	}
	long get_step(double pos1) {
		return (long) ((pos1 - offset) / pos_div_step);
	}
	double get_pos(long step1) {
		return ((double) step1 ) * pos_div_step + offset;
	}
	bool go_pos(double p2g) {
		// ** todo Sicherheitsabfrage nachrüsten
		target = p2g;
		printf("Achse %s go_pos: %lf \n", name.c_str(), target); // debug
		v_fahr = v_std;
		init_ramp();
		go();
		return true;
	}
	bool set_target(double targettoset) {
		// Neue targetposition setzen, noch nicht anfahren
		if (running) {
			printf("Letzter Fahrbefehl ist noch nicht abgeschlossen! \n");
			return false;
		}
		target = targettoset;
		newtarget = true;
		return targettoset;
	}
	bool go_target() {
		if (newtarget) {	
			pthread_mutex_lock(&data_mutex);
			v_fahr = v_std;
			init_ramp();
			go();
			pthread_mutex_unlock(&data_mutex);
			return true;		
		} else return false;
	}
	bool init_ramp() {
		// im Targetmode Ziel anfahren (name der Funktion schlecht gewählt)
		// Letzte Rampe oder Fahrbefehl abgeschlossen?
		if (running) {
			printf("Letzter Fahrbefehl ist noch nicht abgeschlossen! \n");
			return false;
		}
		// Das Ziel bei Position "target" mit Geschwindigkeit "v_std" anfahren.
		// Geschwindigkeit absenken falls Länge der Strecke zu kurz für Erreichen der Sollgeschwindigkeit
		double distanz = fabs(target - get_pos());
		// Maximalgeschwindigkeit bei Beschleunigung und Abbremsen auf einer Rampe der Länge s (v = sqrt(2*0.5*s/a))
		// von der Distanz noch 100 ms mit Mindestgeschwindigkeit abziehen
		double v_rampe;
		a_soll = a_std;
		v_rampe = sqrt((distanz - v_min * 0.1f) / a_soll);
		if (v_fahr > v_rampe) {
			v_fahr = v_rampe;
		}
		// printf("distanz ist %lf, geschwindigkeit ist %lf \n", distanz, v_rampe);
		// sleep(2);
		// Jedoch nicht kleiner als Mindestgeschwindigkeit
		if (v_fahr < v_min) v_fahr = v_min;
		// Richtung
		if ((target - get_pos()) >= 0) dir = POSITIV;
		else dir = NEGATIV;
		// falls kein Fehler
		stop_req = false;
		target_mode = true;
		rs = DATA_OK;
		return true;
	}
	bool go() {
		if (rs == DATA_OK) {
			rs = INIT;
			return true;
		} else {
			printf("Daten der Fahrrampe inkorrekt \n");
			rs = STOP;
			return false;
		}
	}
	bool b_interval(double tt, double la, double lb) {
		// test if tt is between la and lb, also for negative directions
		bool toret;
		if (la <= lb) {
			if ((tt >= la)&&(tt <= lb)) toret = true;
			else toret = false;
		} else {
			if ((tt <= la)&&(tt >= lb)) toret = true;
			else toret = false;
		}
		return toret;
	}
public:
	// axis data
	std::string name;				// Name der Achse
	std::string description;		// zusätzliche Beschreibung
	bool connected;					// Achse angeschlossen/nutzbar
	bool running;					// Achse läuft
	bool test;						// falls true statt Ausgabe der Bits Testdaten ausgeben
	double pos, last_pos;			// berechnete Position, die davor berechnete Position
	double pos_div_step;			// Umrechnung Schrittzahl Position
	double offset;					// Offset
	long steps;						// Aktuelle absolute Schrittzahl
	bool axis_updated;				// erreichte Schluss-Position angezeigt?
	double last_update;				// Zeit letzter Anzeige-update
	// connections
	unsigned int bitmask_pulse;		// Pulsbit
	unsigned int bitmask_dir;		// Richtungsbit
	unsigned int bitmask_spos;		// Anschlagsschalter in positiver Richtung
	unsigned int bitmask_sneg;		// Anschlagsschalter in negativer Richtung
	unsigned int bitmask_sref;		// Anschlagsschalter Referenzposition
	int level_pos_pulse;			// Level des positiven Pulses
	int level_pos_dir;				// Level der positiven Richtung
	int level_on_spos;				// Level spos Schalter ein
	int level_on_sneg;				// Level sneg Schalter ein
	int level_on_sref;				// Level sref Schalter ein
	bool spos_on;					// Zustand schalter pos
	int spos_counter;				// Zählt bei jedem Lesevorgang hoch oder runter erst bei +/- counter_max wird der Pegel neu gesetzt (Antiprell)
	bool sneg_on;					// Zustand schalter neg
	int sneg_counter;				// Antiprell
	bool sref_on;					// Zustand schalter ref
	int sref_counter;				// Antiprell
	const int counter_max = 10;		// Antiprell Integrationsrate
	// drive data
	direction dir;					// Richtung
	// target mode: vor Zielposition abbremsen und mit v_min bis Zielposition fahren
	volatile bool target_mode;			// = true: Targetmode // **volatile? debug ... hilft nicht springt trotzdem manchmal in Fahrmode
	double target;					// Zielposition
	bool newtarget;					// Neue Zeilposition?
	// automatisches Fahren
	bool speed_change_req;			// = true: Sollgeschwindigkeit übernehmen im Speedmode
	// Geschwindigkeit und Beschleunigung
	double v_akt, v_soll, v_min, v_fahr, v_std, v_max; 	// Aktuelle, Soll-, Mindest-, Ausgewählte Fahr-, Standard- und Maximalgeschwindigkeit
	double a_akt, a_soll, a_std, a_max;		// Aktuelle, Standard- und Maximalbeschleunigung
	double bremsweg;				// Maximaler Bremsweg mit aktueller geschwindigkeit
	// Zeiten
	struct timespec ramp_start_time_ts;	// Startzeit der Rampe als timespec Struktur
	double axis_time;				// Zeit seit dem letzten init
	double ramp_start_time;			// Zeit des letzten Zustandswechsels
	double ramp_start_speed;		// Zeit seit dem zustandswechsel
	double s2s;						// abgeleitete aktuelle step to step - Zeit
	// Zustandsvariabele der Rampe
	enum rstat {OFF=0, DATA_OK, INIT, SPEED_UP, SPEED_CONST, SPEED_DOWN, TARGET_SPEED_UP, TARGET_CONST, TARGET_SPEED_DOWN, TARGET_FINE, STOP};
	static const char* rstat_namen[];
	volatile rstat rs;					// Zustandsvariabele zum Fahrbetrieb der Rampe // **volatile? debug ... hilft nicht springt trotzdem manchmal in Fahrmode
	bool stop_req;					// unabhängig vom mode: sofortiges Abbremsen auslösen
	// Zustandsvariabelen der Pulsgenerierung
	enum pulse_stat {PULSE_NONE, PULSE_START, PULSE_UP, PULSE_STOP} pulse;
	double p2ptime;					// Aktueller Abstand zwischen 2 Pulsen
	double pulse_last;				// Zeitpunkt letzter Puls
	double pulse_next;				// geplanter Zeitpunkt nächster Puls
	double pulse_start;				// tatsächlicher Zeitpunkt Pulsstart
	double pulse_stop;				// tatsächlicher Zeitpunkt Pulsstop
	double pulse_length;			// Pulslänge
};

// Konstanten der axis-klasse
const char* axis_data::rstat_namen[] = {"OFF","DATA_OK","INIT", "SPEED_UP", "SPEED_CONST ", "SPEED_DOWN", "TARGET_SPEED_UP", "TARGET_CONST", "TARGET_SPEED_DOWN", "TARGET_FINE", "STOP" };


class drive_data_class
{
public:
	// Konstruktur ruft Elternklassen auf um Daten festzulegen
	drive_data_class()
	{
		run = true;
		refresh = true;
		p_active_axis = NULL;
		axis_end = 0;
	}
	// call this after axis data change
	void axis_update() {
		// Bitmask input updaten
		bm_input_reg = 0;
		int n1;
		for (n1 = 0; n1 < axis_end; n1++) {
			bm_input_reg |= axis_array[n1]->bitmask_spos;
			bm_input_reg |= axis_array[n1]->bitmask_sneg;
			bm_input_reg |= axis_array[n1]->bitmask_sref;
		}
	}
public:
	// Threaddata
	volatile bool run;		// bool if thread should run
	volatile bool refresh;		// refresh the screen if set
	// Functions
	// Stop all axis movement - return false if no axis was running
	bool motor_all_stop() {
		bool b2ret = false;
		long axisnr;
		for (axisnr = 0; axisnr < axis_end; axisnr++) {
			if (axis_array[axisnr]->running) {
				axis_array[axisnr]->stop_req = true;
				b2ret = true;
			}
		}
		return b2ret;
	}
	// active motor set?
	bool motor_active() {
		if (p_active_axis != NULL) return true;
		else return false;
	}
	// Stop selected axis movement - return false if it was not running
	bool motor_active_stop() {
		bool b2ret = false;
		if (p_active_axis != NULL) {
			if (p_active_axis->running) {
				p_active_axis->stop_req = true;
				b2ret = true;
			}
		}
		// while (p_active_axis->running) usleep(1000); // Verhindert screen update beim Bremsen!!!!
		return b2ret;
	}
	std::string motor_active_name() {
		if (p_active_axis != 0) return p_active_axis->name;
		else return std::string("no axis");
	}
	std::string motor_active_description() {
		if (p_active_axis != 0) return p_active_axis->description;
		else return std::string("no axis");
	}
	const char* motor_active_mode() {
		if (p_active_axis != 0) return p_active_axis->modus_str();
		else return std::string("no axis").c_str();
	}
	// start a continuos motion
	bool motor_active_gocont(direction d2g) {
		// Kontinuierliches Fahren
		if ((p_active_axis != NULL)&&(!p_active_axis->running)) {
			p_active_axis->dir = d2g;
			p_active_axis->a_soll = p_active_axis->a_std;
			p_active_axis->target_mode = false;
			p_active_axis->rs = axis_data::INIT;
			return true;
		} else return false;
	}
	// change the speed while running
	bool motor_change_speed(double new_speed) {
		// this include a speed ramp or a stop ...
		if (p_active_axis != NULL) {
			p_active_axis->v_fahr = new_speed * p_active_axis->v_std;
			if (p_active_axis->running) {
				p_active_axis->speed_change_req = true;
			}
		}
		return true;
	}

	// Set the direction for the active motor
	// return true if the motor was running
	bool motor_set_dir(direction d2s) {
		// reprogrammed old version
		bool btr;
		btr = motor_active_stop();
		p_active_axis->dir = d2s;
		motor_dir_out(p_active_axis);
		usleep(1000);
		return btr;
	}
	// write the direction bit of the gives axis according to dir
	bool motor_dir_out(axis_data* p_axis2s) {
		if (p_axis2s != NULL) {
			if (p_axis2s->dir == POSITIV) {
				// Bewegung in positiver Richtung
				// dir_positiv = true; druch direction ersetzen
				// Dir Bit auf low
				if (p_axis2s->level_pos_dir == 0) {
					// Bit auf 0
					// piobyte[p_axis2s->subdev] = piobyte[p_axis2s->subdev] & ~(p_axis2s->bitmask_dir);
					// pio_out(piobyte[p_axis2s->subdev], p_axis2s->subdev); ERSETZEN
					p_gpio->bit_clr(p_axis2s->bitmask_dir);
				} else {
					// Bit auf 1
					// piobyte[p_axis2s->subdev] = piobyte[p_axis2s->subdev] | (p_axis2s->bitmask_dir);
					// pio_out(piobyte[p_axis2s->subdev], p_axis2s->subdev);  ERSETZEN
					p_gpio->bit_set(p_axis2s->bitmask_dir);
				}
			} else if (p_axis2s->dir == NEGATIV) {
				if (p_axis2s->level_pos_dir == 0) {
					// Bit auf 1
					// piobyte[p_axis2s->subdev] = piobyte[p_axis2s->subdev] | (p_axis2s->bitmask_dir);
					// pio_out(piobyte[p_axis2s->subdev], p_axis2s->subdev);
					p_gpio->bit_set(p_axis2s->bitmask_dir);
				} else {
					// Bit auf 0
					// piobyte[p_axis2s->subdev] = piobyte[p_axis2s->subdev] & ~(p_axis2s->bitmask_dir);
					// pio_out(piobyte[p_axis2s->subdev], p_axis2s->subdev);
					p_gpio->bit_clr(p_axis2s->bitmask_dir);
				}
			} else return false;
		}
		return true;
	}
	inline void read_input() {
		// Einlesen
		input_reg = p_gpio->bits_get(bm_input_reg);
		// Daten verteilen
		int n1;
		for (n1 = 0; n1 < axis_end; n1++) {
			// Positive Richtung ...
			// Bit untersuchen und abhängig vom Level eintragen
			if (axis_array[n1]->level_on_spos == 1) {
				if ((input_reg & axis_array[n1]->bitmask_spos) > 0) axis_array[n1]->spos_counter++;
				else axis_array[n1]->spos_counter--;
			} else {
				if ((input_reg & axis_array[n1]->bitmask_spos) == 0) axis_array[n1]->spos_counter++;
				else axis_array[n1]->spos_counter--;
			}
			// Grenzwert erreicht?
			if (axis_array[n1]->spos_counter > axis_array[n1]->counter_max) {
				axis_array[n1]->spos_counter = axis_array[n1]->counter_max;
				axis_array[n1]->spos_on = true;
			} else if (axis_array[n1]->spos_counter < -10) {
				axis_array[n1]->spos_counter = - axis_array[n1]->counter_max;
				axis_array[n1]->spos_on = false;
			}
			// Negative Richtung ...
			if (axis_array[n1]->level_on_sneg == 1) {
				if ((input_reg & axis_array[n1]->bitmask_sneg) > 0) axis_array[n1]->sneg_counter++;
				else axis_array[n1]->sneg_counter--;
			} else {
				if ((input_reg & axis_array[n1]->bitmask_sneg) == 0) axis_array[n1]->sneg_counter++;
				else axis_array[n1]->sneg_counter--;
			}
			// Grenzwert erreicht?
			if (axis_array[n1]->sneg_counter > axis_array[n1]->counter_max) {
				axis_array[n1]->sneg_counter = axis_array[n1]->counter_max;
				axis_array[n1]->sneg_on = true;
			} else if (axis_array[n1]->sneg_counter < -10) {
				axis_array[n1]->sneg_counter = - axis_array[n1]->counter_max;
				axis_array[n1]->sneg_on = false;
			}
			// Referenz - falls implementiert -
			if (axis_array[n1]->bitmask_sref != 0) {
				if (axis_array[n1]->level_on_sref == 1) {
					if ((input_reg & axis_array[n1]->bitmask_sref) > 0) axis_array[n1]->sref_counter++;
					else axis_array[n1]->sref_counter--;
				} else {
					if ((input_reg & axis_array[n1]->bitmask_sref) == 0) axis_array[n1]->sref_counter++;
					else axis_array[n1]->sref_counter--;
				}
				// Grenzwert erreicht?
				if (axis_array[n1]->sref_counter > axis_array[n1]->counter_max) {
					axis_array[n1]->sref_counter = axis_array[n1]->counter_max;
					axis_array[n1]->sref_on = true;
				} else if (axis_array[n1]->sref_counter < -10) {
					axis_array[n1]->sref_counter = - axis_array[n1]->counter_max;
					axis_array[n1]->sref_on = false;
				}
			}
		}
	}
	// Ausgewählte Achse
	axis_data* p_active_axis;
	// Alle Achsen
	static const int axis_array_max = 16;
	axis_data* axis_array[axis_array_max];
	int axis_end = 0;
	// Piobytes
	int input_reg;					// Ergebnis Register-Check
	int bm_input_reg;				// Filter der Inputbits
	// unsigned char piobyte[5];
	// function pointer auf gpio-Klasse
	gpio_driver_class* p_gpio;

};


// Thread zum Anfahren von Zielkoordinaten mittels einer Rampe + gleichzeitiges Fahren mehrerer Achsen
// Erweiterung auf Ändern der Geschwindigkeit / Beschleunigung in einem Fahrmode
void* motor_ramp_driver(void* data) {
	// Task testhalber auf CPU3 festpinnen
	/*
	cpu_set_t mask;
	CPU_ZERO(&mask);
	CPU_SET(3, &mask);
	if (sched_setaffinity(0, sizeof(mask), &mask) == 0) {
		//printf("CPU Affinity set \n");
	} else printf("CPU Affinity NOT set \n");
	*/
	// void-typ aus thread Aufruf umwandeln
	drive_data_class* dd = (drive_data_class*) data;
	// Zeitstruktur
	struct timespec thread_start_time_ts, loop_start_ts, akt_time_ts;
	// struct timespec wakeup_ts, dummy_ts;
	// double p2p_min; p2p_min = 0.1f;
	if (clock_gettime(CLOCK_MONOTONIC, &thread_start_time_ts) != 0) printf("No clock on here! \n");
	if (clock_gettime(CLOCK_MONOTONIC, &loop_start_ts) != 0) printf("No clock on here! \n");
	// Zeiger auf aktuelle Achse
	long axisnr;
	axis_data* p_axis;
	// Endless loop
	while (dd->run)	{
		// Wert aller Motor-Schalter einlesen
		dd->read_input();
		// neue Startzeit loop
		clock_gettime(CLOCK_MONOTONIC, &akt_time_ts);
		// maximale Puls zu Puls-Zeit 0.1s
		// p2p_min = 0.01f; // wann setzen?
		// Alle Achsen durchgehen
		for (axisnr = 0; axisnr < dd->axis_end; axisnr++) {
			// Aktuelle Achse
			p_axis = dd->axis_array[axisnr];
			// Aktiv?
			if (p_axis->connected) {
				// Get the time
				// clock_gettime(CLOCK_MONOTONIC, &akt_time_ts); ** nur nötig falls sich die Zeit während des durchlaufens hier nochmals stark ändert
				
				// Ist Anschlag erreicht? (auch Richtung auswerten)
				if (p_axis->rs == axis_data::SPEED_UP || p_axis->rs == axis_data::SPEED_CONST ||
				        p_axis->rs == axis_data::TARGET_SPEED_UP || p_axis->rs == axis_data::TARGET_CONST) {
					if ((p_axis->sneg_on == 1) && (p_axis->dir == NEGATIV)) {
						p_axis->stop_req = true;
						printf(" Anschlag Neg \n");
					}
					if ((p_axis->spos_on == 1) && (p_axis->dir == POSITIV)) {
						p_axis->stop_req = true;
						printf(" Anschlag Pos \n");
					}
				}
				
				// Lokale Zeit bestimmen seit dem Drivestart, das ist Grundlage für alle Zeitmessungen dieser Achse
				p_axis->axis_time = diff_time(&akt_time_ts, &p_axis->ramp_start_time_ts);
				// Zustandsmaschine Rampe
				switch (p_axis->rs) {
				///////////////////////////////////////////////////////////////////////////////////////
				// OFF und Startmodes
				case axis_data::OFF :
					// Nichts zu tun
					break;
				case axis_data::DATA_OK :
					// Nichts zu tun ...
					break;
				case axis_data::INIT :
					pthread_mutex_lock(&data_mutex);
					///////////////////////////////////////////////////////////////////////
					// Zeitreset: axis_time beginnt ab hier wieder bei Null
					p_axis->ramp_start_time_ts = akt_time_ts;
					p_axis->axis_time = p_axis->ramp_start_time = 0;
					p_axis->ramp_start_speed = 0;
					///////////////////////////////////////////////////////////////////////
					// auf den nächsten Zustand umschalten
					asm("" : "=m" (p_axis->target_mode)); // Tell the compiler to reload the var
					if (p_axis->target_mode == true) {
						if (debug == 1) {
							printf("Target mode: init \n");
							printf("Dist: %lf Mindist: %lf \n", fabs((p_axis->peek_pos() - p_axis->target)), (p_axis->v_fahr * p_axis->v_fahr / p_axis->a_soll));
						}
						// Target-Fahrmodus
						// Prüfen ob Entfernung zum Ziel ausreichend für Rampe - sonst direkt mit v_min anfahren
						if (fabs((p_axis->peek_pos() - p_axis->target)) <= (p_axis->v_fahr * p_axis->v_fahr / p_axis->a_soll)) {
							if (fabs((p_axis->peek_pos() - p_axis->target)) > 0) {
								// mit konstanter Geschwindigkeit Richtung Ziel
								p_axis->a_akt = 0;
								p_axis->v_akt = p_axis->v_min;
								p_axis->rs = axis_data::TARGET_FINE;
								p_axis->ramp_start_time = p_axis->axis_time;
								p_axis->ramp_start_speed = p_axis->v_akt;
								// Richtungsbit setzen
								dd->motor_dir_out(p_axis);
								// Erst in 10 ms den ersten Puls auslösen
								p_axis->pulse_next = 0.010f;
								p_axis->running = true;
							} else {
								// Schon da!
								p_axis->rs = axis_data::STOP;
							}
						} else {
							// mit Rampe Richtung Ziel
							p_axis->v_akt = p_axis->v_min;
							p_axis->v_soll = p_axis->v_fahr;
							p_axis->a_akt = p_axis->a_soll;
							p_axis->rs = axis_data::TARGET_SPEED_UP;
							// Richtungsbit setzen
							dd->motor_dir_out(p_axis);
							// Erst in 10 ms den ersten Puls auslösen
							// (bewirkt Pause zwischen dem Setzen der Richtung und dem ersten Puls)
							p_axis->pulse_next = 0.010f;
							p_axis->running = true;
						}
					} else {
						// Dauer-Fahrmodus
						p_axis->rs = axis_data::SPEED_UP;
						p_axis->v_akt = p_axis->v_min;
						p_axis->v_soll = p_axis->v_fahr;
						p_axis->a_akt = p_axis->a_soll;
						// Richtungsbit setzen
						dd->motor_dir_out(p_axis);
						// Erst in 10 ms den ersten Puls auslösen
						// (bewirkt Pause zwischen dem Setzen der Richtung und dem ersten Puls)
						p_axis->pulse_next = 0.010f;
						p_axis->running = true;
					}
					pthread_mutex_unlock(&data_mutex);
					break;
				//////////////////////////////////////////////////////////////////////////////////////////
				// Dauer-Fahrmodes
				case axis_data::SPEED_UP :
					p_axis->axis_updated = false;
					if (debug == 1) printf("Dauerfahrmodus: Speed_up \n");
					// Beschleunigen bis Soll-Geschwindigkeit erreicht
					if ((p_axis->v_akt >= p_axis->v_soll) || (p_axis->stop_req)) {
						p_axis->a_akt = 0;
						p_axis->ramp_start_time = p_axis->axis_time;
						p_axis->ramp_start_speed = p_axis->v_akt;
						p_axis->rs = axis_data::SPEED_CONST;
						if (debug == 1) printf("Speed_konst \n");
					}
					break;
				case axis_data::SPEED_CONST :
					// Geschwindigkeit beibehalten, falls kein stop oder speed_change_req
					if (p_axis->stop_req) {
						p_axis->a_akt = -p_axis->a_soll;
						p_axis->ramp_start_time = p_axis->axis_time;
						p_axis->ramp_start_speed = p_axis->v_akt;
						p_axis->rs = axis_data::SPEED_DOWN;
						p_axis->v_soll = 0.0f;
						if (debug == 1) printf("STOP-Req: Speed_down \n");
						break;
					}
					if (p_axis->speed_change_req) {
						// Bremsen
						if (p_axis->v_akt > p_axis->v_fahr) {
							p_axis->v_soll = p_axis->v_fahr;
							p_axis->a_akt = -p_axis->a_soll;
							p_axis->ramp_start_time = p_axis->axis_time;
							p_axis->ramp_start_speed = p_axis->v_akt;
							p_axis->rs = axis_data::SPEED_DOWN;
							if (debug == 1) printf("Speed_change: Speed_down \n");
							// request done
							p_axis->speed_change_req = false;
							break;
						}
						// Beschleunigen
						if (p_axis->v_akt < p_axis->v_fahr) {
							p_axis->v_soll = p_axis->v_fahr;
							p_axis->a_akt = p_axis->a_soll;
							p_axis->ramp_start_time = p_axis->axis_time;
							p_axis->ramp_start_speed = p_axis->v_akt;
							p_axis->rs = axis_data::SPEED_UP;
							if (debug == 1) printf("Speed_change: Speed_up \n");
							// request done
							p_axis->speed_change_req = false;
							break;
						}
						// Geschwindigkeit gleich?
						if (p_axis->v_akt == p_axis->v_fahr) {
							// request done
							p_axis->speed_change_req = false;
							break;
						}
					}
					break;
				case axis_data::SPEED_DOWN :
					// printf("SD\n"); //drive_data.refresh = true; // Debug
					// Auf Null bremsen?
					if (p_axis->v_soll == 0) {
						//printf("SD v_soll == 0 \tv_akt %lf \tpos %lf \tlast pos %lf \n", p_axis->v_akt, p_axis->peek_pos(), p_axis->last_pos); // debug
						if (p_axis->v_akt <= p_axis->v_min) {
							p_axis->a_akt = 0;
							p_axis->v_akt = 0;
							p_axis->ramp_start_time = p_axis->axis_time;
							p_axis->ramp_start_speed = p_axis->v_akt;
							p_axis->rs = axis_data::STOP;
							dd->refresh = true;
							p_axis->speed_change_req = false;
							p_axis->stop_req = false;
							if (debug == 1) printf("Stop reached\n");
						}
					} else	{
						// Abbremsen bis v_soll unterschritten
						if (p_axis->v_akt <= p_axis->v_soll) {
							p_axis->a_akt = 0;
							if (debug == 1) printf("Speed_down Speed_change done\n");
							p_axis->speed_change_req = false;
							p_axis->ramp_start_time = p_axis->axis_time;
							p_axis->ramp_start_speed = p_axis->v_soll;
							p_axis->rs = axis_data::SPEED_CONST;
						}
					}
					break;
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Target Fahrmodes
				case axis_data::TARGET_SPEED_UP :
					p_axis->axis_updated = false;
					// Beschleunigen bis Soll-Geschwindigkeit erreicht
					if ((p_axis->v_akt >= p_axis->v_soll) || (p_axis->stop_req)) {
						p_axis->a_akt = 0;
						p_axis->rs = axis_data::TARGET_CONST;
						p_axis->ramp_start_time = p_axis->axis_time;
						p_axis->ramp_start_speed = p_axis->v_akt;
						// Bremsweg für diese Geschwindigkeit berechnen (zum sauberen Anfahren 100 ms mit v_min hinzu)
						p_axis->bremsweg  = 0.5 * p_axis->v_akt * p_axis->v_akt / p_axis->a_soll + p_axis->v_min * 0.1f;
						if (debug == 1) printf("Target speed_up: Bremsweg ist %lf \n", p_axis->bremsweg);
					}
					break;
				case axis_data::TARGET_CONST :
					// Bremsen sobald Ziel in Reichweite oder stop_req
					if (p_axis->stop_req) {
						p_axis->a_akt = -p_axis->a_soll;
						p_axis->ramp_start_time = p_axis->axis_time;
						p_axis->ramp_start_speed = p_axis->v_akt;
						p_axis->rs = axis_data::SPEED_DOWN;
						p_axis->v_soll = 0.0f;
						if (debug == 1) printf("STOP-Req: Speed_down \n");
					}
					if (fabs(p_axis->target - p_axis->peek_pos()) <= p_axis->bremsweg) {
						// Bremsen!
						p_axis->a_akt = -p_axis->a_soll;
						p_axis->ramp_start_time = p_axis->axis_time;
						p_axis->ramp_start_speed = p_axis->v_akt;
						if (debug == 1) printf("Target_const: Ziel fast erreicht: schalte auf Speed_Down \n");
						p_axis->rs = axis_data::TARGET_SPEED_DOWN;
					}
					break;
				case axis_data::TARGET_SPEED_DOWN :
					// Abbremsen bis v_min erreicht (oder unterschritten)
					if (p_axis->v_akt <= p_axis->v_min) {
						p_axis->a_akt = 0;
						p_axis->v_akt = p_axis->v_min;
						if (debug == 1) printf("Target speed down: v_min erreicht Schalte auf target-Fine \n");
						p_axis->rs = axis_data::TARGET_FINE;
						p_axis->ramp_start_time = p_axis->axis_time;
						p_axis->ramp_start_speed = p_axis->v_akt;
					}
					break;
				case axis_data::TARGET_FINE :
					// ganz langsam bis zum Ziel
					if (p_axis->dir == POSITIV) {
						if  ((p_axis->peek_pos() - p_axis->target) >= 0.0f) {
							// printf("GRÖSSER %lf > %lf\n", p_axis->peek_pos(), p_axis->target); DEBUG
							p_axis->a_akt = 0;
							p_axis->v_akt = 0;
							p_axis->target_mode = false;
							p_axis->newtarget = false;
							p_axis->rs = axis_data::STOP;
							dd->refresh = true;
							p_axis->ramp_start_time = p_axis->axis_time;
							p_axis->ramp_start_speed = p_axis->v_akt;
						}
					} else {
						if  ((p_axis->peek_pos() - p_axis->target) <= 0.0f) {
							p_axis->a_akt = 0;
							p_axis->v_akt = 0;
							p_axis->target_mode = false;
							p_axis->newtarget = false;
							p_axis->rs = axis_data::STOP;
							dd->refresh = true;
							p_axis->ramp_start_time = p_axis->axis_time;
							p_axis->ramp_start_speed = p_axis->v_akt;
						}
					}
					if (p_axis->stop_req) {
						p_axis->a_akt = 0;
						p_axis->v_akt = 0;
						p_axis->target_mode = false;
						p_axis->rs = axis_data::STOP;
						dd->refresh = true;
						p_axis->stop_req = false;
					}
					break;
				case axis_data::STOP :
					// target or velocity zero reached - update the screeen
					p_axis->running = false;
					if(!p_axis->axis_updated) {
						dd->refresh = true;
						p_axis->axis_updated = true;
					}
					break;
				}
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Berechnen des nächsten Pulses auf Grund der Beschleunigung und der aktuellen Geschwindigkeit
				if (p_axis->running) {
					if (p_axis->axis_time >= p_axis->pulse_next) {
						// Sicherheitswarnung
						if (p_axis->pulse != axis_data::PULSE_NONE) printf("Puls-Zeitkonflikt\n");
						// Ja - Pulsgenerierung anstoßen
						p_axis->pulse = axis_data::PULSE_START;
						p_axis->pulse_last = p_axis->pulse_next;
						// Zeit für den nächsten Puls berechnen
						// vc = (a_akt * t  + v_akt)
						// Formel t_stp = s_stp / (a_akt  + v_akt)
						// s_stp = p_axis->pos_div_step
						p_axis->v_akt = p_axis->a_akt * (p_axis->axis_time - p_axis->ramp_start_time) + p_axis->ramp_start_speed;
						if (p_axis->v_akt > 0) {
							p_axis->p2ptime = p_axis->pos_div_step / p_axis->v_akt;
							p_axis->pulse_next = p_axis->p2ptime + p_axis->pulse_last;
							// memorize the smallest p2p time
							// if (p2p_min > p_axis->p2ptime) p2p_min = p_axis->pulse_next;
						}
						// Debug
						// printf("TA: %lf PN: %lf PL: %lf \n", p_axis->axis_time, p_axis->pulse_next, p_axis->pulse_last);
					} else {
						// Positionsupdate jede 0,5 mm (oder 1 * Sekunde, wenn eine Achse läuft)
						if (fabs(p_axis->peek_pos() - p_axis->last_pos) > 0.1) {
							dd->refresh = true;
							p_axis->get_pos();
						}
					}
				}
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// pulse control
				switch (p_axis->pulse) {
				case axis_data::PULSE_NONE:
					// Nichts zu tun
					break;
				case axis_data::PULSE_START:
					if (p_axis->test) {
						// axis test modus
						// printf("Pulse on %lf \n", p_axis->axis_time);
					} else {
						// set pulse
						dd->p_gpio->bit_set(p_axis->bitmask_pulse);
					}
					p_axis->pulse = axis_data::PULSE_UP;
					p_axis->pulse_start = p_axis->axis_time;
					// Stepcounter aktualisieren
					if (p_axis->dir == POSITIV) p_axis->steps++;
					else p_axis->steps--;
					break;
				case axis_data::PULSE_UP:
					// Pulszeit abwarten
					if ((p_axis->axis_time - p_axis->pulse_start) < p_axis->pulse_length) {
						break;
					} else {
						if (p_axis->test) {
							// axis test modus
							// printf("Pulse length %lf \n", p_axis->axis_time - p_axis->pulse_start);
						} else {
							// Pulszustand löschen
							dd->p_gpio->bit_clr(p_axis->bitmask_pulse);
						}
						p_axis->pulse_stop = p_axis->axis_time;
						p_axis->pulse = axis_data::PULSE_STOP;
					}
					break;
				case axis_data::PULSE_STOP:
					/*
					// ????
					if ((p_axis->axis_time - p_axis->pulse_stop) < p_axis->pulse_length) {
						break;
					} else {
						p_axis->pulse = axis_data::PULSE_NONE;
						p_axis->stop_req = false; //??
					}
					* */
					p_axis->pulse = axis_data::PULSE_NONE;
					break;
				}
			}
		}
	}
	return NULL;
}


void change_ramp_mode(axis_data* p_ax1, axis_data::rstat mode2set) {
	p_ax1->ramp_start_time = p_ax1->axis_time;
	p_ax1->ramp_start_speed = p_ax1->v_akt;
	p_ax1->rs = mode2set;
}


// helper Functions
double diff_time(struct timespec* pts1, struct timespec* pts2) {
	// Falls Ergebnis negativ ist: Überlauf der Sekunden: korrigieren (wie?)
	double diff;
	diff = (double) (pts1->tv_sec - pts2->tv_sec ) + 1E-9 * (double) (pts1->tv_nsec - pts2->tv_nsec );
	if (diff < 0.0f) {
		// printf("Zeitüberlauf \n");
		// diff = -1.0f;
	}
	return diff;
}

void ts_add_double(struct timespec* p_to_add, double ns2add) {
	double left;
	double right = modf(ns2add, &left);
	p_to_add->tv_sec +=  left;
	p_to_add->tv_nsec +=  right * 1E-9;
}


