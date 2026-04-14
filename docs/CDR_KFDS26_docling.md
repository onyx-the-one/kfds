[Description]
This is a schematic or diagram showing the components of a circuit or system. Here is a detailed description of the components:

- **KFDS26**: This is the name of the component or system.
- **KFD**: This is the brand name of the component or system.
- **S26**: This is the model number of the component or system.
- **O**: This is the operational status or status of the component or system.
- **O-O**: This is the operational status or status of the component or system.
- **O-O-O**: This is the operational status or status of the component or system.
- **O-O-O-O**: This is the operational status or status of the component or system.
- **O-O-O-O-O**: This is the operational status or status of the component or system.
- **O-O-

<!-- image -->

## KFDS26 Critical design review

Bratislava, 1. apríl 2026

## Obsah

Obsah .......................................................................................................................................................  2

| 1. Predstavenie                                                                                                                                                                      | tímu.............................................................................................................................3              |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| 2.                                                                                                                                                                                   | Harmonogram...................................................................................................................................5 |
| 2.1 Prvá fáza: návrh a plánovanie (december -                                                                                                                                        | január)...........................................................................5                                                             |
| 2.2 Druhá fáza: stavba prototypu (január - marec).................................................................................5                                                  |                                                                                                                                                 |
| 2.3 Tretia fáza: testovanie a ladenie (marec - apríl) ...............................................................................6                                               |                                                                                                                                                 |
| 2.4 Štvrtá fáza: príprava na finálnu prezentáciu (apríl) ...........................................................................6                                                |                                                                                                                                                 |
| 3. Prehľad misie ...................................................................................................................................7                                |                                                                                                                                                 |
| 3.1 Primárna misia................................................................................................................................7                                  |                                                                                                                                                 |
| 3.2 Sekundárna misia............................................................................................................................7                                    |                                                                                                                                                 |
| 3.3 Terciárna misia...............................................................................................................................8                                  |                                                                                                                                                 |
| 4. Riziká a očakávané komplikácie                                                                                                                                                    | ........................................................................................................9                                       |
| 5. Mechanický návrh konštrukcie                                                                                                                                                      | ........................................................................................................11                                      |
| 5. Mechanická konštrukcia                                                                                                                                                            | .................................................................................................................11                             |
| 5.1 Technické požiadavky pre mechanickú konštrukciu: .....................................................................11                                                         |                                                                                                                                                 |
| 5.2 Základný opis sondy:.....................................................................................................................11                                      |                                                                                                                                                 |
| 5.3 Výber materiálov: ..........................................................................................................................11                                   |                                                                                                                                                 |
| 5.4 Vývoj sondy:                                                                                                                                                                     | ..................................................................................................................................12            |
| 6. Elektronická konštrukcia .................................................................................................................13                                      |                                                                                                                                                 |
| 6.1 Všeobecný dizajn ..........................................................................................................................13                                    |                                                                                                                                                 |
| 6.1.1PCB doska - návrh a štruktúra ....................................................................................................13                                            |                                                                                                                                                 |
| 6.1.2PCB doska - spájkovanie a realizácia .........................................................................................14                                                |                                                                                                                                                 |
| 6.1.3 Anténa pre GNSSmodul(NEO-M9N-00B)...................................................................................14                                                         |                                                                                                                                                 |
| 6.2 Sekundárna misia..........................................................................................................................15                                     |                                                                                                                                                 |
| 6.3 Terciárna misia..............................................................................................................................15 6.3.1 Kvantová distribúcia kľúča | .....................................................................................................16                                         |
| 6.3.2 Cieľ a význam terciárnej misie ................................................................................................16                                              |                                                                                                                                                 |
| 6.3.3 Teoretický základ kvantovej distribúcie kľúčov .......................................................................17                                                       |                                                                                                                                                 |
| 6.3.4 Aplikácia protokolu BB84 v CanSate.......................................................................................17                                                    |                                                                                                                                                 |
| 6.4 Napájanie......................................................................................................................................19                                |                                                                                                                                                 |
| 6.5 Komunikačný systém ....................................................................................................................20                                        |                                                                                                                                                 |
| 7. Softvér.................................................................................................................................................21                        |                                                                                                                                                 |
| 7.1 Hardvérové periférie......................................................................................................................21                                     |                                                                                                                                                 |
| 7.2 Štruktúra                                                                                                                                                                        |                                                                                                                                                 |
| úloh FreeRTOS                                                                                                                                                                        | ...............................................................................................................22                               |

| 7.3 Komunikačný protokol- PolySense .............................................................................................22        | 7.3 Komunikačný protokol- PolySense .............................................................................................22            |
|--------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 7.4 Konfigurácia rádiového modulu LoRa ............................................................................................23      | 7.4 Konfigurácia rádiového modulu LoRa ............................................................................................23          |
| 7.5 Spracovanie príkazov a stavový automat misie..............................................................................23           | 7.5 Spracovanie príkazov a stavový automat misie..............................................................................23               |
| 7.6 Vývojové prostredie a build systém................................................................................................23   | 7.6 Vývojové prostredie a build systém................................................................................................23       |
| 8.                                                                                                                                         | Návratový systém ...........................................................................................................................24 |
| 9.                                                                                                                                         | Pozemná stanica ............................................................................................................................25 |
| 9.1Pozemná stanica - softvér ...........................................................................................................25 | 9.1Pozemná stanica - softvér ...........................................................................................................25     |

Promo ........................................................................................................................................  26

3

10.

## 1. Predstavenie tímu

Tím  KFDS26  sa  skladá  zo  šiestich  študentov  francúzskej  bilingválnej  sekcie  Gymnázia  Metodova (Bratislava):

| Menáčlenov tímu abecedne   | Úloha                                                                                              |
|----------------------------|----------------------------------------------------------------------------------------------------|
| Martin Babka               | Expert na počítačové systémy, zodpovedný za elektrické systémy, program a komunikáciu.             |
| Viliam Bednárik            | Hlavný programátor, oblasť expertízy JAVA a insight na viacerých C++aC#projektoch                  |
| Ján Besson                 | Zodpovedný za stavbu satelitu,HW, PR a propagáciu                                                  |
| EmaBeňová                  | Inžinierka mechanickej konštrukcie a výroby                                                        |
| Alexandra Butašová         | Zodpovedná za terciárnu misiu (kvantová kryptografia), grafika, komunikácia s organizátormi súťaže |
| Alica Nogová               | KonštrukciaHW,pomocpri elektrickom dizajn e                                                        |

Nášmu tímu tiež pomáhajú:

| Učitelia         |                                                          |
|------------------|----------------------------------------------------------|
| Terézia Jindrová | Vyučujúca fyziky na francúzskej sekcii gymnázia Metodova |

| Mentori       |                                                                    |
|---------------|--------------------------------------------------------------------|
| Daniel Buchta | Expert na kvantové technológie a AI, matematik                     |
| Lukáš Krkoška | Produktový manažér pre 3IPK, odborník na kryptografiu a blockchain |

## 2. Harmonogram

## 2.1 Prvá fáza: návrh a plánovanie (december - január)

V prvej fáze sa zameriavame na detailný návrh všetkých komponentov satelitu, výberu senzorov a návrhu architektúry jednotlivých modulov. Súčasťou tejto fázy je  aj analýza viacerých konštrukčných riešení a výber najvhodnejšieho prístupu na základe stanovených požiadaviek.

Táto fáza predstavuje základ pre ďalší vývoj, pričom jej výstupy budú následne overené v prototypovej fáze. Tím sa stretáva pravidelne 1 až 2 -krát týždenne, pričom návrhy sú priebežne konzultované a upravované.

| Hlavné úlohy a kroky:                                                                          | približný čas využitý na realizáciu:   |
|------------------------------------------------------------------------------------------------|----------------------------------------|
| Dokončenie detailného návrhu modulu                                                            | 4 mesiace                              |
| Výber a objednanie potrebných súčiastok (motory, senzory, batérie, kamera a ďalšie komponenty) | 2 mesiace                              |
| Príprava podkladov pre Critical Design Review (CDR)                                            | 33 hodín                               |
| Návrh architektúry simulácie kvantovej distribúcie kľúča (QKD) - terciárna misia.              | 6 hodín                                |

| Propagačné ciele:                                                                                                                      | približný čas využitý na realizáciu:   |
|----------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------|
| Redizajn tímového loga a vizuálnej identity vrátane tímových tričiek                                                                   | 5 hodín                                |
| A ktívna správa oficiálneho Instagramového účtu tímu (vrátane pravidelného zverejňovania príspevkov o tíme, misii a priebehu projektu) | Pravidelne: 20mintýždenne              |

## 2.2 Druhá fáza: stavba prototypu (január - marec)

Táto fáza nadväzuje na návrhovú fázu a slúži na experimentálne overenie zvolených riešení. Po zostavení elektroniky  sa  celý  systém  naprogramuje,  čo  umožňuje  správne  fungovanie  všetkých  jeho  častí.  Po dokončení elektroniky sa podľa nej upraví mechanická časť satelitu z minulého roka. Na základe výsledkov testovania budú návrhy iteratívne upravované pred prechodom do finálnej fázy.

| Hlavné úlohy a kroky:                                                                         | približný čas využitý na realizáciu:                 |
|-----------------------------------------------------------------------------------------------|------------------------------------------------------|
| Zostaveniemechanickej časti satelitu (inštalácia kolies, montáž motorov, upevnenie senzorov)  | 10 hodín (veľká časť už bola hotová z minulého roka) |
| Testovanie pohybu na rôznych povrchoch, sklonoch a terénoch a zabezpečenie stability systému, | 10 hodín                                             |
| Návrh a implementácia prvej verzie elektronickej dosky pre riadenie motorov a komunikáciu     | 4 mesiace                                            |
| Programovanie a integrácia jednotlivých systémov vrátaneQKD                                   | 100 hodín                                            |
| Integrácia atestovanie senzorov(plyny, teplota, vlhkosť, kamera a pod.)                       | idk                                                  |

| Propagačné ciele:                                        | približný čas využitý na realizáciu:   |
|----------------------------------------------------------|----------------------------------------|
| Z dieľanie príspevkov dokumentujúcich vývoj a testovanie | Pravidelne: 20mintýždenne              |
| Prezentácia demoverzie satelitu (napr. na školách)       | 1deň                                   |
| Príprava na účasť na Dni otvorených dverí (DOD)          | 2 dni pred                             |

Commented [BM1]: zacali sme dakedy v nov-dec cca ak sa nemylim cize take 4 mesiace?

Commented [BM2]: davam cca cas, neviem presne kolko to trvalo plus sme nesli na maximum ihned

Commented [JB3]: toto som prepisal lebo najskor sme robili lelektroniku a az potom bude mechanika podla elektroniky takze pozrite cit o dava zmysel

Commented [BM4]: prebieha od zaciatku planovania kfds26 cize podla mna to takto sedi. taktiez je to ta ista doska na komunikaciu, gnss a senzory, a neovlada motory.

## 2.3 Tretia fáza: testovanie a ladenie (marec -apríl)

V tejto fáze prebieha finálne testovanie všetkých modulov a systémov s cieľom overiť ich spoľahlivosť a pripravenosť na súťažné podmienky. Fáza slúži ako validačný krok pred odovzdaním projektu. V prípade identifikácie nedostatkov budú systémy upravené a opätovne testované.

| Hlavné úlohy a kroky:                                       | Odhadčasupotrebného na realizáciu:   |
|-------------------------------------------------------------|--------------------------------------|
| finálne testovanie všetkých modulov                         | 20 hodín                             |
| test zhodenia sondy pomocouRClietadla                       | 2 hodiny                             |
| testovanie komunikačných a senzorickýchsystémov             | 2 hodiny                             |
| Programovanie a integrácia jednotlivých systémov vrátaneQKD | idk                                  |
| identifikácia a odstránenie nedostatkov                     | Záleží odpočtu nedostatkov           |

| Propagačné ciele:                                          | Odhadčasupotrebného na realizáciu:   |
|------------------------------------------------------------|--------------------------------------|
| komunikácia s médiami a školskými platformami              | 3 hodiny                             |
| prezentácia finálnej verzie satelitu na sociálnych sieťach | 3 hodiny                             |
| vytvorenie sumarizačného videa projektu                    | 6 hodín                              |

## 2.4 Štvrtá fáza: príprava na finálnu prezentáciu (apríl)

Po  dokončení  testovania  a  ladenia  budeme  pripravovať  finálnu  dokumentáciu  a prezentáciu  pre  Final Design Review.

| Hlavné úlohy a kroky:                                          | Odhadčasupotrebného na realizáciu:   |
|----------------------------------------------------------------|--------------------------------------|
| príprava finálnej dokumentácie,                                | 40 hodín                             |
| príprava prezentácie (vizuály, funkcionalita, výsledky testov) | 10 hodín                             |

Commented [BM5]: tuna bude vsetko trvat asi maximum dostupneho casu, je to konstantny proces

Commented [BM6]: znova kolko casu budeme mat toklo vyuzijeme, predpokladam ze cas nazvys nebude

## 3. Prehľad misie

## 3.1 Primárna misia

Cieľom  primárnej  misie  je  splniť  kritériá  stanovené  organizátorom  súťaže,  konkrétne  zabezpečiť,  aby CanSat  počas  zostupu  spoľahlivo  meral  teplotu  a  tlak  vzduchu.  Namerané  údaje  sú  spracovávané  a odosielané  prostredníctvom  rádiovej  telemetrie  na  pozemnú  stanicu  s  frekvenciou  minimálne  raz  za sekundu počas celého letu.

Dáta  sú  prenášané  v  pravidelných  paketoch,  ktoré  obsahujú  merané  hodnoty  a  základné  kontrolné informácie na overenie správnosti prenosu. Tým sa zabezpečuje spoľahlivá komunikácia medzi satelitom a pozemnou stanicou aj v podmienkach možného rušenia.

Po pristátí  je  cieľom  primárnej  misie  analyzovať  získané  dáta,  vrátane  výpočtu  výšky  na  základe  tlaku vzduchu, a prezentovať výsledky vo forme prehľadných grafov, ako napríklad závislosť výšky od času a teploty od výšky.

## 3.2 Sekundárna misia

Naším cieľom je otestovať využiteľnosť a funkčnosť multifunkčnej sondy, ktorá sa po pristátí transformuje  na  rover.  Tento  dizajn  umožňuje  jej nasadenie v rôznych misiách a podmienkach, pričom sa dokáže jednoducho prispôsobiť požiadavkám konkrétnej misie.

Transformačný  systém  jej  umožňuje  pokračovať  v činnosti aj po dopade, vykonávať merania na povrchu a efektívnejšie interagovať s prostredím.

Sonda jazdiaca po povrchu pri vykonávaný misie

<!-- image -->

<!-- image -->

## Etapy transformácie:

1. vypustenie sondy z požadovanej výšky,
2. otvorenie kolesa a padáku,
3. rozvinutie padáku a opornej tyče, o ktorú sa pri pohybe sonda zapiera,
4. pristátie, otvorenie druhého kolesa a odpojenie padáku.

Kladieme dôraz na jednoduchosť dizajnu a minimalizáciu možných bodov zlyhania a snažíme sa vyberať ľahko  dostupné  a  lacné  súčiastky (s  ohľadom  na  kvalitu).  Vďaka  tomuto  prístupu  chceme  sprístupniť využitie  výskumných  sond  širšiemu  spektru  používateľov,  čím  uľahčíme  realizáciu  vedeckých  misií  a rozšírime ich dostupnosť pre nové výskumné tímy. Cieľom sekundárnej misie je otestovať transformáciu počas pádu a n ásledne schopnosť pohybovať sa v rôznych terénoch po pristátí.

Commented [BM7]: @Jan Marcel Besson   Študent neviem ci mate nejaku novsiu/lepsiu fotku, ak ano tak dajte tu ak nie tak toto je fajn podla mna

Commented [JB8R7]: nn podla mna najvystiznejsia

## Praktické využitie sondy:

| NaZemi:   | NaZemi:                                                                                                                                                                                                            | Vo vesmíre:                                                                                                                                                                                                                                                                    |
|-----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| - - - - - | vyhľadávanie nezvestných osôb v krízových oblastiach, detekcia nebezpečných plynov, monitorovanie kvality ovzdušia a pôdy, zber environmentálnych dát v neprístupných oblastiach, monitorovanie a kontrola územia, | - prieskum atmosféry a povrchu iných planét, - mapovanie terénu a geologických štruktúr, - testovanie technológií v podmienkach mikrogravitácie. ( Navyužitie vo vesmíre by musela byť sonda vyrobená z odlišných materiálov, no základný princíp fungovania zostáva rovnaký.) |

Vďaka nízkej cene a jednoduchej replikovateľnosti môže byť naša sonda v budúcnosti nasadzovaná vo veľkom počte, napríklad v podobe koordinovaných 'rojov'. Takýto roj sond by mohol byť využitý napríklad pri pátraní po nezvestných osobách v krízových situáciách -naraz by sa mohli vypustiť desiatky zariadení vybavených  infračervenými  senzormi  a  ďalšími  čidlami  na  detekciu  ľudskej  prítomnosti.  Sondy  by navzájom  komunikovali,  koordinovali  svoj  pohyb  a  umožnili  rýchle  a  efektívne  prehľadanie  rozsiahlej oblasti.

V komerčnej sfére by bolo možné ponúkať rôzne služby od merania environmentálnych parametrov,  až po dlhodobé monitorovanie konkrétnych lokalít.

## 3.3  Terciárna misia

V rámci terciárnej misie budeme modelovať, ako  by mohli malé  satelity v  budúcnosti bezpečne komunikovať pomocou kvantových princípov.

Tento  ciel  bude  realizovaný  prostredníctvom softvérovej simulácie kvantovej distribúcie kľúča (QKD), konkrétne protokolu BB84 (prvého kvantového kryptografického protokolu)  medzi  CanSatom    a  pozemnou stanicou.

Bližšie o terciárnej misii vid. v bode 6.3.

<!-- image -->

Commented [JB9]: @everyone neciem ci toto dava zmysel co som sem napisal?

Commented [BM10R9]: hej moze byt len mierna jazykova uprava

## 4. Riziká a očakávané komplikácie

Naša misia čelí viacerým technickým aj prevádzkovým výzvam, ktoré je nevyhnutné podrobne analyzovať a efektívne vyriešiť. Od samotného začiatku projektu sa sústreďujeme na optimalizáciu dizajnu tak, aby spĺňal všetky požiadavky, minimalizoval riziká a zároveň zabezpečil čo najvyššiu spoľahlivosť.

Trup sondy musí byť vyrobený z materiálov, ktoré sú pevné, no zároveň ľahké, aby odolali nárazu pri pristátí a pritom neprekročili stanovené hmotnostné limity.

Kolesá musia dokázať absorbovať náraz a zároveň zabezpečiť dostatočnú trakciu na rôznorodom povrchu. Okrem toho sa musia zmestiť do obmedzeného priestoru na bokoch sondy a po pristátí sa spoľahlivo roztvoriť.

| Riziko                                    | Pravdepodobnosť        | Dopad                                                        | Riešenie                                                                                                            |
|-------------------------------------------|------------------------|--------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------|
| Prekročenie váhy                          | Nepravdepodobné        | Diskvalifikácia                                              | Optimalizácia komponentov a ich váhy                                                                                |
| Prekročenie rozmerov                      | Nepravdepodobné        | Diskvalifikácia                                              | Použitie šablóny a presné navrhnutie a vytlačenie pomocou3Dtlačiarne                                                |
| Rozbitie trupu                            | Pravdepodobné          | Neškodné až fatálne                                          | Použitie pateriálu PETG Carbon Filled alebo ASAza účelom spevnenia celkovej komštrukcie                             |
| Dážď                                      | Nepravdepodobné        | Veľký                                                        | Utesnenie otvorov a škár                                                                                            |
| Dopadnastrom                              | Nepravdepodobné        | Fatálny, nevieme sa pohnúť ďalej                             | Ide o jav, ktorý je do značnej miery ovplyvnený vonkajšími faktormi mimonašej kontroly                              |
| Dopadnakameň                              | Nepravdepodobné        | Fatálne poškodenie                                           | Ide o jav, ktorý je do značnej miery ovplyvnený vonkajšími faktormi mimonašej kontroly                              |
| Dopaddovody                               | Nepravdepodobné        | Fatálny                                                      | Ide o jav, ktorý je do značnej miery ovplyvnený vonkajšími faktormi mimonašej kontroly                              |
| Silný vietor                              | Pravdepodobné          | Fatálny                                                      | Ide o jav, ktorý je do značnej miery ovplyvnený vonkajšími faktormi mimonašej kontroly                              |
| Roztrhnutie padáku                        | Nepravdepodobné        | Fatálny                                                      | Použitie Ripstop latky cey ktorú sa trhliny nerozširujú po celej dlžke šva                                          |
| Roztrhnutie švu padáku                    | Nepravdepodobné        | Fatálny                                                      | Spevnenie švov                                                                                                      |
| Roztrhnutie šnúry padáku                  | Vysoko nepravdepodobné | Fatálny                                                      | Výber správneho materiálu, ktorý vydrží požadovanú silu                                                             |
| Roztrhnutie spojov medzi padákom a trupom | Nepravdepodobné        | Stredne veľký                                                | Spevnenie spojov a dokladne otestovanie                                                                             |
| Neotvorenie padáku                        | Nepravdepodobné        | Fatálny                                                      | Správne poskladanie a dizajn                                                                                        |
| Nenafúknutie kolies                       | Nepravdepodobné        | Veľký                                                        | Správny výber špongie, ktorá sa nafúkne aj po stlačení                                                              |
| Nesprávna rýchlosť padáku                 | Pravdepodobné          | Veľký                                                        | Správny tvar a veľkosť vďaka výpočtom a testovaniu                                                                  |
| Otvorenie kolies príliš skoro             | Nepravdepodobné        | Fatálny, ak sa otvoria príliš skoro,nemôže sa uskutočniť pád | Namontované poistky, ktoré držia kolesá stlačené                                                                    |
| Voda v kolesách                           | Nepravdepodobné        | Veľký                                                        | Plášť/ impregnácia kolies                                                                                           |
| Prepichnutie baterky                      | Veľmi nepravdepodobné  | Fatálny                                                      | Obozretná manipulácia                                                                                               |
| Zlomenie PCBdosky                         | Nepravdepodobné        | Fatálny                                                      | Obozretná manipulácia                                                                                               |
| Nefunkčnosť motorov                       | Nepravdepodobné        | Veľký                                                        | Ak sú správne zapojené do obvodu, tak sa môževyskytnúť len riziko konštrukčnej chyby, ktorú vieme len minimalizovať |

Commented [BM11]: je to cervene ale pride mi vsetko urobene

Commented [JŠ12]: @Babka Martin @Alexandra Butašová -Študent dava toto zmysel ze proste ked sa aj vytvroi diersa alebo trhloina nerozsiruje sa dalej?

Commented [BM13R12]: hej presne, potom je jediny problem ze prechadza vzduch a ze nedostatocne spomaluje ale snad mame nejaku rezervu.

|                                      |                 |                           | kúpou kvalitných komponentov                                                                                                                     |
|--------------------------------------|-----------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| Pokazenie ložiska                    | Nepravdepodobné | Veľký                     | Dobre nadizajnované ložiská zabezpečujú, že budú fungovať a nerozbijú sa                                                                         |
| Rozbitie trupu pri práci             | Nepravdepodobné | Veľký                     | Opatrné manévrovanie                                                                                                                             |
| Pokazenie kamery                     | Nepravdepodobné | Veľký                     | Ak sú správne zapojené do obvodu, tak sa môževyskytnúť len riziko konštrukčnej chyby, ktorú vieme len minimalizovať kúpou kvalitných komponentov |
| Pokazenie tlakomera                  | Nepravdepodobné | Veľký                     | Ak je správne zapojený do obvodu, tak sa môževyskytnúť len riziko konštrukčnej chyby, ktorú vieme minimalizovať len kúpou kvalitného komponentu  |
| Nenájdenie satelitu                  | Pravdepodobné   | Fatálny                   | Sledovanie GPSapozície počas pádu                                                                                                                |
| NefungovanieGPS                      | Nepravdepodobné | Veľký                     | Ak je správne zapojená do obvodu, tak sa môževyskytnúť len riziko konštrukčnej chyby, ktorú vieme minimalizovať len kúpou kvalitného komponentu  |
| Nefungovanie programov               | Nepravdepodobné | Fatálny                   | Vopred otestované funkcie a ich fungovanie                                                                                                       |
| Vybitie baterky                      | Nepravdepodobné | Fatálny                   | Nabitie pred misiou                                                                                                                              |
| Vytečenie baterky                    | Nepravdepodobné | Fatálny                   | Výber kvalitného komponentu a opatrná manipulácia                                                                                                |
| Zaseknutie sa počas jazdenia         | Pravdepodobné   | Veľký                     | Presné riadenie a sledovanie trasy                                                                                                               |
| Strata kontaktu s riadiacou stanicou | Pravdepodobné   | Diskvalifikácia           | Vopred otestované spojenie a prenos informácií                                                                                                   |
| Zlomenie opornej tyče                | Nepravdepodobné | Veľký                     | Výber kvalitnej opornej tyče                                                                                                                     |
| Zapichnutie sa tyče do zeme          | Nepravdepodobné | Fatálny                   | ide o náhodu y nič sa s tým nedá robiť                                                                                                           |
| Nevysunutie sa tyče                  | Nepravdepodobné | Fatálny                   | Upravenie dizajnu a minimalizovanie rizika poruchy                                                                                               |
| Satelit sa nevrátikam chceme         | Pravdepodobné   | Veľký                     | Pôvodné autonómne riadenie smevymenili za manuálne                                                                                               |
| Nefunkčnosť ovládania                | Pravdepodobné   | Veľký                     | Vopred otestované prenosy dát a fungovanie potrebných systémov na ovládanie                                                                      |
| Nadmerný šumpri simuláciiQKD         | Nepravdepodobné | Zlyhanie terciárnej misie | ImplementáciaFEC                                                                                                                                 |
| Vlhkosť v elektronike                | Nepravdepodobné | Fatálny                   | Utesnenie otvorov a škár na trupe                                                                                                                |
| Pokazenie výškomeru                  | Nepravdepodobné | Veľký                     | Ak je správne zapojená do obvodu, tak sa môževyskytnúť len riziko konštrukčnej chyby, ktorú vieme minimalizovať len kúpou kvalitného komponentu  |
| Zamotanie šnúr padáku                | Pravdepodobné   | Fatálny                   | Nastavenie správnej dĺžky šnúr na padáku                                                                                                         |

## 5. Mechanický návrh konštrukcie

Riešenie konštrukcie má pre nás veľký význam. Nakoľko je realizácia sekundárnej misie hlboko prepojená s konštrukciou satelitu ,  je  potrebné venovať jej  v  mnohých smeroch veľkú mieru dôrazu a detailnosť.

## 5.1 Technické požiadavky pre mechanickú konštrukciu:

Trup sondy musí byť vyrobený z odolných, no ľahkých materiálov , ktoré vydržia náraz pri pristátí a zároveň neprekročia hmotnostné limity.

Kolesá musia  absorbovať  náraz  a  zároveň  poskytnúť  dostatočnú trakciu na  rôznych  typoch  terénu. Zároveň sa musia zmestiť do obmedzeného priestoru na  bokoch  sondy  a  byť  schopné  sa  po  pristátí správne rozvinúť.

| Požiadavka              | Zdôvodnenie                                                   |
|-------------------------|---------------------------------------------------------------|
| Minimalizácia hmotnosti | Aby hmotnosť celej sondy neprekročila limit 350g              |
| Dodržanie rozmerov      | Plechovky115mm výška ×66mmpriemer                             |
| Preťaženiesondy20G      | Aby konštrukcia a všetky mechanizmy vydržali náraz pri dopade |
| Minimalizácia ceny      | Aby smesplnili náš ciel dostupnosti a neprekročili limit 500€ |

## 5.2 Základný opis sondy:

Naša sonda má tvar plechovky, na ktorú je z oboch strán pripevnené jedno koleso. Zo stredu smerom k zemi vyčnieva oporná tyč, ktorá zabraňuje tomu, aby sa sonda pri pohybe otáčala okolo vlastnej osi, keďže má len dve kolesá.

## Konštrukcia sondy musí byť navrhnutá tak, aby spĺňala nasledovné funkcie:

- -Efektívne tlmila náraz pri pristátí a chránila vnútorné komponenty pred vplyvmi prostredia
- -Obsahovala mechanizmy na otvorenie padáku a jeho bezpečné odpojenie po pristátí
- -Umožnila transformáciu na rover systém nafukovacích kolies a vysúvaciu opornú tyč

## 5.3 Výber materiálov:

## Hlavné kritéria výberu filamentu:

- -Cenová dostupnos ť
- -Odolnosť
- -S chopnosť zachovať pri výrobe prec ízny tvar a ve ľkosť

Experimentovali sme s PETG, PETG Carbon Fiber a ASA, pretože pon ú kali najlepší pomer odolnosti a ceny. Tento rok máme taktiež , vďaka grantu s Tatrabanky , špičkovú tlačiareň Prusa Core one+ , ktorá dokáže tlačiť aj filamenty , ktoré potrebujú kontrolu okolitej teploty ako ASA alebo nylon.

Po  vykonaní  testov, však  na  vačšinu  komponentov  použijeme  filament  ASA,  pretože  jeho  odlnosť  je porovnatelná s PETG Carbon fiber no je lahšie dostupný a hlavne si dokáže zachovať tvar na vytvorenie precíznych súčiatok. Pri Carbonových filamentoch boli častokrát súčiatky menej presné. Výber ASA tiež p odporuje náš cieľ ľahkej opakovateľnosti a dostupnosti.

Porovnanie typov  filamentov, ich odolnosti a ceny:

Commented [JŠ14]: @Alexandra Butašová -Študent neveim ci atale davame tieto tabulky poziadavok ku kazdej sekcii alebo zvlasť takze kludne ich vymaz ak zavadzaju

## Hardware Diagram

The diagram shows the components and their connections in a typical electronic system. Here is a detailed description of the components and their connections:

### Components
1. **PETG**:
   - **Type**: PETG (Polycarbonate Fiberglass)
   - **Manufacturer**: PETG
   - **Model**: 101.15
   - **Price**: 25
   - **Manufacturer**: Matterhacker
   - **Model**: 101.15
   - **Price**: 20
   - **Manufacturer**: Matterhacker
   - **Model**: 101.15
   - **Price**: 20
   - **Manufacturer**: Matterhacker
   - **Model**: 101.15
   - **Price**: 20
   - **Manufacturer**: Matterhacker
   - **Model**: 101.15
   - **Price**: 20

<!-- image -->

| Typ filamentu      | Maximálna záťaž pred zlomením (kg)   | Cena (€/kg)   | Dáta o maximálnej záťaži pochádzajú zo stránky Matterhacker (https://www.matterhackers.com/news/filament- strength-testing). Testovanie prebiehalo vešaním závaží na 3Dtlačenú karabínu . Ceny sú orientačné.   |
|--------------------|--------------------------------------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PETG               | 101,15                               | ~ 25          | Dáta o maximálnej záťaži pochádzajú zo stránky Matterhacker (https://www.matterhackers.com/news/filament- strength-testing). Testovanie prebiehalo vešaním závaží na 3Dtlačenú karabínu . Ceny sú orientačné.   |
| PLA                | 83,46                                | ~ 20          | Dáta o maximálnej záťaži pochádzajú zo stránky Matterhacker (https://www.matterhackers.com/news/filament- strength-testing). Testovanie prebiehalo vešaním závaží na 3Dtlačenú karabínu . Ceny sú orientačné.   |
| ASA                | 128,82                               | ~ 20          | Dáta o maximálnej záťaži pochádzajú zo stránky Matterhacker (https://www.matterhackers.com/news/filament- strength-testing). Testovanie prebiehalo vešaním závaží na 3Dtlačenú karabínu . Ceny sú orientačné.   |
| Polycarbonate      | 188,24                               | ~ 40          | Dáta o maximálnej záťaži pochádzajú zo stránky Matterhacker (https://www.matterhackers.com/news/filament- strength-testing). Testovanie prebiehalo vešaním závaží na 3Dtlačenú karabínu . Ceny sú orientačné.   |
| Nylon              | 72,57                                | ~ 35          | Dáta o maximálnej záťaži pochádzajú zo stránky Matterhacker (https://www.matterhackers.com/news/filament- strength-testing). Testovanie prebiehalo vešaním závaží na 3Dtlačenú karabínu . Ceny sú orientačné.   |
| Nylon+Carbon Fiber | 161,02                               | ~ 100         | Dáta o maximálnej záťaži pochádzajú zo stránky Matterhacker (https://www.matterhackers.com/news/filament- strength-testing). Testovanie prebiehalo vešaním závaží na 3Dtlačenú karabínu . Ceny sú orientačné.   |
| PETG+Carbon Fiber  | 120,66                               | ~ 30          | Dáta o maximálnej záťaži pochádzajú zo stránky Matterhacker (https://www.matterhackers.com/news/filament- strength-testing). Testovanie prebiehalo vešaním závaží na 3Dtlačenú karabínu . Ceny sú orientačné.   |

TPU98A filament používame tam, kde je nutná ohybnosť materiálu . Testovali sme rôzne stupne trdosti 'Shore Hardness' a tiež TPE , čo je iný typ flexibilného filamentu , no ten má horšie mechanické vlastnosti ako TPU a je menej odolný. TPU alebo termoplastický polyuretán stupňa tvrdosti 98A sme vybrali, lebo ponúka ideálny  balans medzi ohybnosťou a odolnosťou, vďaka čomu naše súčiastky dokážu udržať požadovaný tvar, no sú flexibilné.

Na rýchle  prototypovanie, kde nie je prioritou mechanická odolnosť, používame PLA filament  pre jeho ľahšiu a rýchlejšiu tlač v porovnaní s ASA alebo PETG.

## Tlačiarne a softvér:

- -Prusa Core One+ -veľmi presna tlačiar e ň vhodná na finálne výtlačky s filamentov ktoré potrebujú kontorlu okolitej teploty.
- -Bambulab A1 mini -dostatočne kvalitná tlačiareň na prototypy s PLA a na tlač TPU
- -3D modely tvoríme v Fusion 360 -používame ho s licenciou pre študentov.

## 5.4 Vývoj a fungovanie sondy:

Pri  návrhu  sme  sa riadili  princípmi "design for manufacturing", čo znamená, že všetky komponenty sú navrhnuté s ohľadom na jednoduchú výrobu a môžu byť vytlačené bez špeciálnych úprav. Tento prístup zaručuje ľahkú sériovú výrobu sondy a jej ľahkú modifikác iu pre rôzne misie.

Jednotlivé súčiastky sondy sú navrhnuté ako samostatne vymeniteľné segmenty (kolesový segment a telo), čo umožňuje ich jednoduchú výmenu v prípade poškodenia alebo potreby úpravy. Segmentovanosť nám zároveň umožnila rýchlo testovať a vylepšovať rôzne mechanické riešenia bez nutnosti prepracovania celej konštrukcie. Segment tela satelitu a segment kolies sme vyvíjali a upravovali nezávisle od seba.

Zakladne  koncepty  fungovania  sme  si  overili  už  počas  minulorocneho  finale  kedy  cela  mechanická konštrukcia  prežila  pád  aj  so  zle  otvoreným  padákom.  Sústredili  sme  sa teda  na  jej  zdokonalenie  a upravenie pre umiestnenie novej elektroniky.

## Vylepšenia oproti minulému roku:

- Redesign držiaku ktorý drží koleso zavreté počas letu
- o Zjednodušenie zatváranie a zvýšenie spolahlivosti
- Zjednodušenie prístupu k baterke
- o Umožnuje výmenu aj bez otvorenia celého satelitu
- Redesign mechanizmu opornej tyče
- o Spolahlivejšie uchytenie počas letu a zjednodušenie skladania
- Vytvorenie držiakov na plošné spoje a elektroniku
- o Zabezpečujú uchytenie komponentov no aj efektivnejšie využitie priestoru
- Vytvorenie centrálneho vypínaču

## Porovnanie mechanickej konštrukcie:

## Verzia použitá počas finále minulého roka

The image is a technical diagram or schematic of a satellite dish system. The diagram is divided into several sections, each representing different components of the satellite dish system. Here is a detailed description of each section:

1. **Satellite Dish System Description**:
   - **Primary Components**:
     - **Satellite**: A satellite dish is a device that converts radio waves into visible and infrared signals. It consists of a dish, a receiver, and a power source.
     - **Satellite Dish**: The dish is a large, flat, and rectangular object that is mounted on a flat surface. It is designed to receive and transmit signals from the satellite dish.
     - **Satellite Receiver**: The receiver is a small, cylindrical device that receives the signals from the satellite dish. It is typically made of plastic or metal and has a built-in antenna.
     - **Power Source**: The power source is a small, cylindrical device that provides the power to the satellite dish. It

<!-- image -->

## Aktuálna verzia

The device has four wheels.

<!-- image -->

## Kolesový segment:

V prvej časti sa zamerame na kolesové segmenty ktoré zabezpečujú najdoležitejšie a zároveň najzložitejšie funkcie.

## Funkcie:

1. otvorenie padáku
2. otvorenie kolies

3. nafúknutie kolies
4. otáčanie kolies
5. Ochrana satelitu zo strán pri dopade a počas misii na zemi

Kolesový segment sa skladá s 3 základných častí . Ložisko zabezpečuje plynulé otáčanie kolesa a prepojenie s motorom a satelitom, samotného kolesa a d ržiak u ktorý sa na lozisko upevní zakrútenim a drží koleso zatvorené počas letu a umožnuje jeho rýchl e otvorenie v správnej chvíli.

## Základné časti:

The hardware diagram, schematic, or pinout is a diagram that shows the connections and relationships between various components of an electrical system. Here is a detailed description of the hardware diagram, schematic, or pinout:

### Hardware Diagram

- **Cylinder**: The central object in the diagram is a large, flat, yellow-colored object. This is the base of the cylinder.
- **Sphere**: The object to the right of the cylinder is a small, blue, cube-shaped object. This is the sphere.
- **Frame**: The frame is a flat, rectangular piece of material that holds the objects together.
- **Hole**: The object in the center of the frame is a small, black hole.
- **Hole**: The object in the center of the frame is a small, black hole.
- **Hole**: The object in the center of the frame is a small, blue hole.
- **Hole**:

<!-- image -->

## 1. Ložisko

Pozostáva z dvoch častí :

1. Vonkajšia časť -pevne pripojená k telu satelitu.
2. Vnútorná  časť -voľne rotujúca,  poháňaná malým  DC  motorčekom  so  zabudovanou prevodovkou .

Motor  je  prepojený  s vnútornou  rotujúcou  časťou cez veľké  vnútorné  ozubenie ,  ktoré  zabezpečuje prevodový pomer 7:4 . Tento pomer zvyšuje krútiaci moment , čo umožňuje efektívny pohyb sondy aj v náročnejšom teréne.

Prierez ložiska pohlaď zospodu:

Ložisko

-

či erne

Koleso

-

biele

Držiak

- modrý

<!-- image -->

## Ložiskový systém:

Na zabezpečenie hladkého pohybu vnútornej časti sú medzi dvoma hlavnými časťami umiestnené dva rady oceľových guľôčok, ktoré sú vybrané zo skateboardových ložísk a doplnené plastovými rozostupovačmi. Týmto spôsobom vytvárame vlastné plastové ložisko, ktoré umožňuje plynulé otáčanie kolies sondy aj bez použitia klasického oceľového ložiska, ktoré by bolo na naše použitie príliš ťažké.

Výroba vlastného ložiska nám tiež poskytuje možnosť úprav podľa našich potrieb napríklad môžeme do konštrukcie  integrovať  vnútorné  ozubenie,  čím  získame  kompaktné  riešenie  pohonu  bez  potreby dodatočných komponentov.

Pôvodne sme testovali jednoradové ložisko, no to sa ukázalo ako nedostatočne pevné na absorbovanie nárazov pri pristátí. Preto sme pridali dvojradový systém, ktorý výrazne zlepšil mechanickú odolnosť.

Celá základňa je vyrobená z ASA filamentu, čo zabezpečuje jej odolnosť. Je vytlačená ako jeden celok technológiou 'print -inplace', pričom tlač je zastavená na vloženie guľôčok a separátorov. Tento postup eliminoval potrebu dodatočnej montáže, čím sa minimalizovali potenciálne slabé miesta v konštrukcii.

Fialová - vnútorná otáčajúca sa časť s ozubením po vnútornom obvode

Ružová - vonkajšia časť pripevnená k satelitu

Oranžová - malé ozubené koliesko pripojené k motoru

<!-- image -->

<!-- image -->

The blue circular object in the image is a bolt. It has a hole in the center and is designed to be inserted into a surface. The hole is circular and has a smooth surface. The bolt is blue and has a smooth, shiny surface. There are four bolts on the bolt, which are silver in color. The bolts are evenly spaced and are connected to each other.

<!-- image -->

## 2. Koleso

## Pohľad z boku na otočnú základňu

Fialová -vnútorná otáčajúca sa časť s ozubením po vnútornom obvode

Ružová -vonkajšia časť pripevnená k satelitu

Oranžová -guličky a plastové separátory umiestnené v dvoch radoch vložené počas tlače

## Pohlaď z vrchu na otočnú základňu

Fialová -vnútorná otáčajúca sa časť s ozubením po vnútornom obvode

Ružová -vonkajšia časť pripevnená k satelitu

Oranžová -guličky a plastové separátory vložené počas tlače

Pozastavenie 3D tlače na vloženie guličiek

Koleso musí byť počas letu uschované v malom priestore satelitu a po jeho otvorení sa musí nafúknuť do požadovanej veľkosti, aby umožňovalo pohyb. Na jeho nafúknutie sme pôvodne chceli použiť chemickú reakciu,  no  tá  bola  príliš  nevyspytateľná.  Preto  sme  zvolili  pasívne  nafukovanie,  ktoré  zabezpečuje špongiový  materiál,  z  ktorého  je  vyrezané.  Tento  materiál  sa  nielen  dokáže  stlačiť  do  veľmi  malého priestoru, aby počas letu nezaberal miesto, ale hneď po jeho vypustení sa špongia samočinne nafúkne bez nutnosti akéhokoľvek pomocného mechanizmu.

Týmto  spôsobom  opäť  všetko  čo  najviac  zjednodušujeme  a  eliminujeme  možné  body  zlyhania.  Toto špongiové koleso je upevnené na satelit pomocou suchého zipsu, čo umožňuje jeho jednoduchú výmenu a odmontovanie. Keďže sa špongia však chce prirodzene vždy nafúknuť, potrebujeme niečo, čo ju počas letu bude držať pevne stlačenú v satelite.

## 3. Držiak na koleso

Držiak na koleso uchováva špongiu počas letu. Koleso sa do neho pred štartom natlačí a po zatočení sa uzamkne na ložisku.

V jednom z držiakov, ktorý je väčší, je uschovaný aj padák, a na druhom je padák priviazaný. Oba držiaky sú navzájom prepojené padákovým lankom, aby sa po odpojení nestratili, ale zostali visieť na padáku.

Umiestnenie kolesa v držiaku namontovanom na ložisku:

<!-- image -->

Modrá (hore) -

držiak

Čiern a (dole) -

ložisko

Červený ovál - priestor na uschovanie kolesa a na jednej strane aj padáku počas letu

Zelená -

suchý zips držiaci koleso

Po odistení držiaku zatočením do opačnej strany sa tento následne roztvor í a koleso sa nafúkne.

**Image Description:**

The image is a technical diagram of a blue plastic part, which appears to be a part of a larger structure. The part is shown in an exploded view, with each section labeled and clearly labeled. The part is shown in a circular shape, with a small protrusion at the top. The protrusion is blue and appears to be made of a material that is hard and has a smooth surface.

**Description:**

1. **Top Section:**
   - **Shape:** The part is a circle with a small protrusion at the top.
   - **Color:** The protrusion is blue.

2. **Middle Section:**
   - **Shape:** The part is a square with a small protrusion at the top.
   - **Color:** The protrusion is blue.

3. **Bottom Section:**
   - **Shape:** The part is a cube with a small protrusion at the top.
   - **Color:** The protrusion is

<!-- image -->

Zatvorený držiak

## Segment tela satelitu:

Táto časť satelitu slúži prevažne na ochranu a držanie elektronických súčiastok.

## Funkcie:

6. Ochrana elektronických komponentov
7. Jednoduchý prístup k elektronickým komponentom
8. Jednoduchý spôsob výmeny batérie
9. Otvorenie a uchytenie opornej tyče

Kolesá sa na telo pripevnujú jednoduchým mechanizmom zatlačenia a otočenia čo umožnuje lahké sprístupnenie batérky a hlavného vypínača bez potreby čokolvek skrutkovať.

## Pripevnenie kolesového segmentu k telu:

The diagram shows a section of a television screen, with three panels. Panel 1 is labeled "Zlatacenie teola do kolesa," which translates to "Zlatacenie teola of the Kolesa." Panel 2 is labeled "Ocenie kolesa," which translates to "Ocenie of the Kolesa." Panel 3 is labeled "Pevne spojie dvoch cast!" which translates to "Pevne spojie dvoch cast!" The diagram also includes a red arrow indicating a 3-degree turn.

<!-- image -->

Prístup k hlavnému vypína ču a batérke:

Otvorený držiak

The image shows a technical diagram of a rocket engine, specifically a turbofan engine. The engine is depicted in a simplified, two-dimensional view, with the engine's components labeled and labeled in a manner that is consistent with standard technical drawings.

### Engine Components:

1. **Propeller**:
   - **Shape**: A large, cylindrical engine with a central hub and a propeller at the top.
   - **Diameter**: The diameter of the propeller is approximately 10 meters.

2. **Propeller Motor**:
   - **Shape**: A large, cylindrical engine with a central hub and a propeller at the top.
   - **Diameter**: The diameter of the propeller is approximately 10 meters.

3. **Propeller Motor**:
   - **Shape**: A large, cylindrical engine with a central hub and a propeller at the top.
   - **Diameter**: The diameter of the propeller is approximately 10 meters.

4. **

<!-- image -->

Telo satelitu sa skladá s jedného hlavného oddelenia v ktorom sú uschované všetky súčiatky okrem palubného počítača. Každá súčiatka má vnútry vytvorený držiak alebo otvor presne na svoju velkosť vďaka čomu drží na mieste. Toto oddelenie sa dá pre lepší prístup celé otvoriť.

## Otvorenia satelitu pre prístup k elektronike:

The image depicts a basic schematic or diagram of a satellite or a satellite-like object. The objects in the image are labeled with their respective names:

1. **Satellite**: The satellite is depicted as a large, metallic object with a circular shape. It has a large, circular dish-like structure at the top, which is likely used for transmitting signals or data. The dish is connected to a larger, rectangular structure at the bottom, which is likely the main body of the satellite.

2. **Sector**: The satellite is divided into two sectors, each with a smaller rectangular structure. The smaller sector is located at the top of the satellite, and the larger sector is located at the bottom.

3. **Sector Connection**: The satellite is connected to the larger sector via a series of smaller rectangular connectors. These connectors are likely used to connect the satellite to other satellites or to other pieces of equipment.

4. **Sector Connection**: The satellite is also connected

<!-- image -->

Druhé menšie oddelenie sa nachádza na otačnej strane tela a v ňom je uschovaný len palubný počítač. Na prístup k tomuto oddeleniu stačí vytiahnuť dva železné kolíky ktoré su v satelite len

zatlačené a netreba odmontovávať kolesá. Týmto pádom je počítač dobre chránený no zároveň je k nemu lahký prístup pre účel programovania.

## Prístup k druhému oddeleniu:

This is a schematic diagram, schematic, or pinout of a solar photovoltaic (PV) cell.

<!-- image -->

## Oporná tyč:

Tyč sme sa rozhodli vytlačiť na 3D tlačiarni z TPU98A filamentu, pretože musí byť schopná zrolovať sa okolo tela sondy počas letu. Tento materiál sa však po uvoľnení veľmi dobre  vracia do pôvodného tvaru a tak sa sama vystrie.

Tyč je plochá z oboch strán, čo jej umožňuje ľahko sa ohnúť okolo sondy.

Počas letu je plochá strana tyče rovnobežná s osou otáčania satelitu, a keďže je v tomto smere ohybná, nedokázala by dostatočne satelit počas pohybu podopierať.

The diagram depicts a cylindrical object, likely a piston, with two circular openings. The object is labeled "Tyt plochou sronou rovobezne" which translates to "Tyt plochou sronou rovobezne" in English. The openings are labeled "k o al" which means "on the left" and "o al" which means "on the right."

The object is shown in a horizontal orientation, with the openings pointing towards the left side. There are arrows indicating the direction of the movement of the object. The arrows are red and green, with red indicating the direction of the movement and green indicating the direction of the object's rotation.

The object is labeled with the text "Tyt plochou sronou rovobezne" in English. The text is in a sans-serif font, and the letters are in uppercase. The arrows are also in uppercase, and the

<!-- image -->

Môžeme vidieť, že šípky sú na ľavom obrázku v rovnakom smere, čo by znamenalo, že pri pôsobení sily na tyč by sa iba naspäť ohla. Preto bolo potrebné, aby sa tyč pri premene na rover otočila o 90 stupňov.

The image depicts two circular objects, which appear to be part of a socket or socket box. These objects are labeled as "Pocas letu" and "Poznare na rover." The sockets are designed to fit into sockets, and the sockets are labeled with the text "Poznare na rover" which translates to "Poznare on the rover."

### Description of Objects:

1. **Sockets**:
   - **Pocas letu**: This is the socket that is being described.
   - **Poznare na rover**: This is the socket that is being described.

2. **Sockets**:
   - **Pocas letu**: This socket fits into the socket labeled "Poznare na rover."
   - **Poznare on the rover**: This socket fits into the socket labeled "Poznare na rover."

3. **Sockets**:
   - **Pocas letu

<!-- image -->

Preto  sme  vytvorili mechanizmus, ktorý  tyč  po  jej  uvoľnení  a  vystretí  pomocou  gumičky otočí  o  90 stupňov, a tým jej plochú stranu orientuje kolmo na os otáčania satelitu. Okrem toho sa tyč zasunie do tela satelitu a uzamkne sa v tejto polohe vďaka magnetom upevnených z každej strany tyče.

## Mechanizmu s otvárania tyče :

The diagram shows a series of steps to install a car battery. The first step is to remove the battery from the battery tray. The second step is to remove the battery from the battery tray and insert it into the battery compartment. The third step is to insert the battery into the battery compartment and secure it. The fourth step is to secure the battery in place. The fifth step is to remove the battery from the battery tray and insert it into the battery compartment.

<!-- image -->

## Fungovanie mechanizmu:

Koleso je nadizajnované tak aby všetky jeho funkcie spúšťal len jeden motor ktorý je následne využitý aj na pohon. To nam umožnuje šetriť priestor a hmotnosť a tiež eliminuje dalšie body zlýhania ktoré by mohli motory navyše predstavovať.

Pred letom je držiak zatlačený a zakrútený do ložiska a koleso je stlačené medzi nimi. Padákové lahk á idú popri tele satelitu od jedného držiaku k druhému. Jedno z nich ide popod zrolovanú opornú tyč.

<!-- image -->

Držiak upevnený na ložisku pomocou vytrčajúcich nožičiek

Zelená -

nožička

Červenou - vyznačený smer vkladania nožičiek na držiaku do ložiska na jeho upevnenie

<!-- image -->

Celý mechanizmus sa spúšťa roztočením kolesa v smere hodinových ručičiek (teda v opačnom smere ako sa držiak otáča pri zaisťovaní).

(1.) Prvé na začiatku pádu sa roztočí koleso v ktorom je uschovaný padák. Pri roztočení motora sa vnútorná otočná časť ložiska (na obrázkoch fialová)  pohne a zatlačí do ložiskových nožičiek ktoré sa posunú  až dokym sa neodistia s ložiska. V tej chvíli sa prvý držiak odpojí od satelitu a zostane visieť len na šnúrke. Vďaka tomu sa rozprestrie padák ktorý je vytlačený nafukujúcim sa kolesom a toto prvé koleso sa tiež uplne nafukne.

- (2.)  trhnutie padákovej šnúry tiež uvolní opornú tyč ktorá sa počas zvyšku pádu vystrie .

(3.) po pristáti sa roztočí druhé koleso čo odistí aj druhy držiak a umožni jeho nafúknutie. Po tomto sú obydva držiaky a padák uplne odpojené od satelitu a satewlit sa može volne pohybovať.

Pripomenutie etáp premeny na rover:

<!-- image -->

## Etapy transformácie:

5. vypustenie sondy z požadovanej výšky,
6. otvorenie kolesa a padáku,
7. rozvinutie padáku a opornej tyče, o ktorú sa pri pohybe sonda zapiera,
8. pristátie, otvorenie druhého kolesa a odpojenie padáku.

Prierez kolesovýcm segmentom na vyzualizáciu mechanizmu (pohlad z vrchu):

The diagram shows a circular object with a red circle as the center. There are two vertical lines extending from the center to the edges of the object. These lines are labeled "vex" (vex) and "vex-vex" (vex-vex). There are also two horizontal lines extending from the center to the edges of the object. These lines are labeled "vex-vex" and "vex-vex-vex" (vex-vex-vex-vex-vex-vex-vex-vex-vex-vex-vex).

<!-- image -->

## Testovanie a validácia:

Okrem toho že zákaldný princíp rozdelenia na segmenty sme mali otestovaný z minulého roku sme aj novú verziu satelitu intenzívne testovali.

Najskôr sme celý satelit bez padáku zhadzovali z výšky 3 metre na betónovú podlahu a následne sme ho celý aj s testovacou elektronikov vyhodili zo 4. poschodia zo školy na školský dvor. Oba testy satelit prežil a elektronika v ňom tiež.

Schopnosť presúvať sa po rôznych terénoch sme testovali jazdami. Satelit sme počas nich ovládali ovládačom na RC lietadlá kedže sme ešte nemali funkčnú elektroniku.

## Povrchy na ktorých bol satelit testovaný a jeho schopnosť pohybu na nych:

| povrch                                  | pohyb                                                        |
|-----------------------------------------|--------------------------------------------------------------|
| Sneh                                    | Veľmi dobrá trakcia. Bezproblémový pohyb                     |
| Krátka tráva                            | Veľmi dobrá trakcia. Bezproblémový pohyb                     |
| Dlhá tráva (vyššia ako kolesá satelitu) | Občasné zasekávanie a problém s otáčaním                     |
| Linoleum                                | Bezproblémový pohyb. V prípade prekážky mierne prešmykovanie |
| Asfalt                                  | Veľmi dobrá trakcia. Bezproblémový pohyb                     |

Z výsledkov sme usúdili že schopnosť pohybu satelitu je dostatočná kedže vo vačšine vonkajších prostredí sa pohyboval bezproblémovo.

## 6. Elektronická konštrukcia

## 6.1 Všeobecný dizajn

- Všetky komponenty sú ovládané centrálnym počítačom ESP32 -WROOM-32U
- DJI kamera je jediná výnimka -ide o úplne samostatný diel, nepripojený k ESP32-WROOM-32U
- K ESP32-WROOM-32U sú pripojené:
- o BME688 (senzor tlaku, teploty, vlhkosti a plynov)
- o ICM-42688-P (IMU -akcelerometer a gyroskop)
- o LoRa komunikačný modul

## 6.1.1 PCB doska -návrh a štruktúra

Satelit využíva PCB dosku (printed circuit board -plošný spoj ) PolySenseV1, navrhnutú v programe KiCad.

## Hlavné integrované obvody na doske:

- ICM-42688-P

IMU - akcelerometer a gyroskop

- BME688

Senzor tlaku, teploty, vlhkosti a plynov

- E22-900M22S

LoRa rádiový modul na komunikáciu

- NEO-M9N-00B

GNSS modul na polohovanie

- 3 kondenzátory

Filtrácia a stabilizácia napájania

## Vrstvová štruktúra štvorvrstvová doska:

1. vrstva -signálová vrstva
2. vrstva -napájacia vrstva
3. vrstva -prázdna (zámerne nevyužitá)
4. vrstva -súvislé uzemnenie (ground plane)

Toto  rozloženie  zabezpečuje  dobrú  integritu  signálov,  stabilnú  referenčnú  zem  a  zároveň  jednoduchú výrobu. Z dôvodu vysokej ceny a výrobnej náročnosti buried vias boli použité štandardné priechodné vias. Aby sa zabránilo nechcenému prepojeniu so zemnou rov inou, boli v ground plane vytvorené izolačné otvory (antipads), vďaka čomu vias prepájajú iba prvú a druhú vrstvu. Na doske sa nachádzajú tri kondenzátory určené  na  filtráciu  a  stabilizáciu  napájania.  Súčasťou  návrhu  je  aj  12 -pinový  header,  ktorý  umožňuje pripojenie externého mikrokontroléra a ďalších periférií počas testovania a integrácie systému. Celý návrh PCB bol realizovaný s dôrazom na prehľadnosť, minimalizáciu rušenia a praktickú realizovateľnosť výroby.

A schematic diagram is a graphical representation of electrical components and their interconnections. The diagram is divided into several sections, each representing a different part of the circuit. Here is a detailed description of the components and their connections:

1. **Central Board**:
   - **Top Left**: Contains the circuit board, which is a rectangular structure with a grid of lines.
   - **Top Right**: Contains the power supply, which is a large, rectangular structure with a grid of lines.
   - **Bottom Left**: Contains the input and output pins, which are located on the right side of the board.
   - **Bottom Right**: Contains the output pins, which are located on the left side of the board.

2. **Control Poles**:
   - **Top Left**: Contains the control poles, which are small, rectangular structures with a grid of lines.
   - **Top Right**: Contains the control poles, which are larger, rectangular structures with a grid of lines.
   - **Bottom

<!-- image -->

Návrh  PCB  v  programe  KiCad  s vyznačenými vodivými cestami (traces). Modrá plocha predstavuje súvislú uzemnú rovinu (ground plane),  červené  cesty  znázorňujú hlavné signálové traces na vrchnej vrstve  a  zelené  prepojenia  patria druhej signálovej vrstve.

## 6.1.2 PCB doska -spájkovanie a realizácia

Sedem komponentov, vrátane troch blokovacích kondenzátorov, bolo pripájkovaných metódou pretavenia spájkovacej pasty (reflow soldering). Na presné nanášanie pasty boli použité na mieru vyrobené kovové  šablóny  (stencils).  Komponenty  boli  manuálne  umiestňova né  pomocou  pinziet  pod lupou -najmenší kontakt na doske meral 0,40 × 0,40 mm a  najväčší 2,0  ×  0,8  mm.  Pretavovací proces prebehol pomocou teplovzdušnej stanice pri 350 °C, čím vznikli spoľahlivé spájkované spoje. Pre účely testovania boli namiesto 12-pino vého konektora dočasne použité jumper kábliky, čo  umožňuje flexibilnejšie prepájanie počas integrácie systému.

The hardware diagram, schematic, or pinout is a black coin with the number 2 on it.

Osadená PCB doska s komponentmi; vedľa 2 -eurová minca pre porovnanie veľkosti

<!-- image -->

## 6.1.3 Anténa pre GNSS modul (NEO-M9N-00B)

Pre GNSS modul NEO-M9N00B sme zvažovali použitie dvoch typov antén : pasívnej a aktívnej . Na základe porovnania kľúčových technických a praktických parametrov sme zostavili rozhodovaciu maticu (decision matrix) , ktorá hodnotí oba varianty podľa piatich kritérií: hmotnosť, rozmer, spotreba elektrickej energie, účinnosť a cena. Každému kritériu bola priradená váha podľa jeho dôležitosti pre danú misiu.

|                | Hmotnosť   | Rozmer   | Spotreba el.   | Účinnosť   | Cena   | TOTAL   |
|----------------|------------|----------|----------------|------------|--------|---------|
| Váha kritérií  | 2          | 3        | 1              | 5          | 2      |         |
| Pasívna anténa | 5/5        | 4/5      | 4/5            | 2/5        | 4/5    |         |
| Výsledok       | 10         | 12       | 4              | 10         | 8      | 44/65   |
| Aktívna anténa | 2/5        | 2/5      | 2/5            | 5/5        | 2/5    |         |
| Výsledok       | 4          | 6        | 2              | 25         | 4      | 41/65   |

Výsledok  rozhodovacej  matice  je veľmi  vyrovnaný ,  pasívna  anténa  získala 44  z  65  možných  bodov a aktívna anténa 41 z 65 bodov . Rozdiel predstavuje iba 3 body, čo naznačuje, že oba varianty sú pre danú aplikáciu porovnateľne vhodné. Pasívna anténa mierne prevyšuje aktívnu najmä vďaka nižšej hmotnosti, menším  rozmerom  a  nižšej  spotrebe  energie,  čo  sú  pre  platformu  CanSat  dôležité  faktory.  Konečné rozhodnutie bude prijaté po dôkladnejšom otestovaní oboch variantov v reálnych podmienkach.

## 6.2 Sekundárna misia

Sekundárnou misiou systému KFDS26 je demonštrácia premeny klasického CanSatu na plne funkčný semiautonómny rover po pristátí a zber dát v mieste dosadnutia. Cieľom je overiť, či integrovaný podvozok, akčné členy a senzorová sústava dokážu po náraze spoľahlivo prejsť do 'rover' režimu a vykonať krátku prieskumnú a meraciu sekvenciu.

Primárne merania sekundárnej misie budú zahŕňať:

- -meranie zrýchlenia a uhlových rýchlostí pomocou IMU ICM-42688P počas nárazu, vysunutia kolies a pohybu rovera, čo umožní analyzovať dynamiku pristátia a jazdy,
- -záznam polohy pomocou GNSS modulu NEOM9N (trajektória od pristátia po koniec jazdy, rýchlosť, výškový profil),
- -environmentálne merania tlaku, teploty, vlhkosti a VOC plynov senzorom BME688 v režime blízko pri povrchu,
- -meranie  stavových  veličín  rovera  (PWM  motorov,  prúd  motorov,  napätie  batérie)  na vyhodnotenie energetickej náročnosti a spoľahlivosti pohonu,
- -prípadne (ak bude modul osadený) meranie úrovne ionizačného žiarenia Geigerovou trubicou LND 712 a vzdialeností ultrazvukovým senzorom MB1240 na demonštráciu rozšíriteľnosti nákladu.

Na  prenos  dát  sa  využije  paketový  protokol  s  binárnym  payloadom,  ktorý  obsahuje  časovú  pečiatku, prepočítané hodnoty zo senzorov (teplota, akcelerácia, poloha, stav motorov)  a  CRC 16-CCITT kontrolu pre  overenie  integrity.  Tým  sa  umožní  podrobná  rekonštrukcia  priebehu  sekundárnej  misie z uložených telemetrických záznamov na pozemnej stanici.

Očakávaným výsledkom je preukázanie, že:

- -systém  dokáže  autonómne  detegovať  pristátie  (náraz  &gt;15  g),  aktivovať nitinolový   mechanizmus, bezpečne rozvinúť kolesá a prejsť do jazdného stavu,
- -rover je schopný vykonať riadený pohyb (napr. niekoľko desiatokmetrov) bez nadmerného  prehrievania motorov a s akceptovateľným odberom z 21700 Li -ion batérie,
- -dvojitý komunikačný systém (dual LoRa + FPV) zabezpečí takmer nulovú stratu paketov v podmienkach priamej viditeľnosti a umožní kontinuálny dohľad nad roverom.

Vedecký a demonštračný význam misie spočíva v tom, že ide o realistický model malého planetárneho rovera  v  extrémne  obmedzenom  objeme  a  hmotnosti  CanSatu.  Projekt  ukazuje,  ako možno  v študentských  podmienkach  integrovať  moderné  senzory,  r obustnú  rádiovú   komunikáciu,  autonómne riadenie a mechanizmus premeny na rover do jedného  systému. Takto získané dáta môžu slúžiť ako základ na porovnanie rôznych konštrukčných riešení (typy kolies, aktorov, algoritmov riadenia) a ako referenčný demonštrátor pre budúce tímy, ktoré budú vyvíjať pokročilejšie 'rover -style' sekundárne misie.

## 6.3 Terciárna misia

Realizácia terciárnej misie spočíva v tom, že pozemná stanica (odosielateľ) a CanSat (prijímateľ) si vytvoria spoločný šifrovací kľúč prostredníctvom simulácie kvantového kryptografického protokolu BB84.

Na základe tohto kľúča CanSat zašifruje vopred definovanú správu a odošle ju pozemnej stanici. Pozemná stanica následne správu dešifruje pomocou rovnakého kľúča a vyhodnotí chybovosť prenosu porovnaním s pôvodným obsahom správy. Táto výmena bude realizovaná softvérovo v programovacom jazyku C++ a komunikáci a bude prebiehať prostredníctvom rádiového spojenia.

## 6.3.1  Kvantová distribúcia kľúča

Kvantová distribúcia kľúča (QKD) je  metóda tvorby kryptografického kľúča, ktorá v reálnych  systémoch využíva kvantové stavy jednotlivých fotónov, napríklad ich polarizáciu .

Výsledkom procesu je spoločný bezpečný kľúč, ktorý sa  následne používa v štandardných šifrovacích algoritmoch (nie šifrovaná správa ako taká).

Commented [BM15]: sanca ze to tam realne dame je ze 1x10 ⁻ ⁸ % (velmi objektivna analyza). ale hej mozeme to tam mat napisane lebo mozne to je

Commented [JŠ16R15]: watahelly

Commented [JŠ17R15]: LND=lobotomizacna trubica

Commented [BM18]: som zabudol ze toto sme planovali. ale moze to byt teoreticky

Commented [JŠ19R18]: co vobec je nilonovy mechsnizmus

Commented [BM20R18]: to je ten drat co ked ho zahrejes resp. pustis don prud tak meni tvar. v nasom pripade vypusti koleso nie mechanicky ale elektronicky. to boli stare plany ale asi z toho nic?

Commented [BM21]: mechaniku neviem, jano alebo dakto toto musi potvrdit

Commented [JŠ22R21]: ehmmmm trosku to prepisem nech to dava zmzysel so zvyskom

V priebehu protokolu si odosielateľ a prijímateľ navzájom vymieňajú fotóny nesúce určitú polarizáciu, ktoré sú merané a porovnávané.

Bezpečnosť tvorby kľúča je zaručená tým, že prípadný odpočúvateľ naruší kvantové stavy jednotlivých fotónov, čo môže byť následne detekované.

Význam QKD spočíva v tom, že jeho bezpečnosť nie je založená na výpočtovej náročnosti matematických problémov, ale na fyzikálnych  zákonoch.  V  kontexte  rozvoja  kvantových  počítačov,  ktoré  môžu  ohroziť súčasné kryptografické štandardy, predstavuje QKD technológiu umožňujúcu dlhodobú ochranu citlivých komunikačných kanálov.

The diagram shows a projector with a lens and a boom attached to it. The lens is pointed at a target. The target is a square with a grid of numbers. The projector is pointed at the target.

<!-- image -->

## 6.3.2 Cieľ a význam terciárnej misie

V  dnešnej dobe  už prebiehajú  experimentálne  implementácie  QKD  v satelitných  a  vesmírnych komunikačných  systémoch,  čo  poukazuje  na  dynamický  rozvoj  tejto  technológie  a  jej  prechod  z laboratórneho  prostredia  do  reálnych  aplikačných  scenárov.  Vesmírne  prostre die  pritom  predstavuje jednu z kľúčových platforiem pre rozvoj kvantovej komunikácie, najmä v kontexte budovania globálnych kvantových sietí  a  kvantového  internetu.  Tento  smer  vývoja  je  zároveň  výrazne  podporovaný  vysokými investíciami zo strany štátov aj súkromného sektora, čo podčiarkuje jeho strategický význam. V budúcnosti existuje  vysoká  pravdepodobnosť,  že  podobné  technológie  sa  stanú  súčasťou  bežnej  komunikačnej infraštruktúry.

V  súčasnosti  je  však  stále  reálna  hardvérová  implementácia  QKD  pri  malých  satelitoch  typu  CanSat technologicky a  finančne  náročná,  vzhľadom  na  cenu  a  citlivosť  potrebných  komponentov.  Preto  sa  v našom projekte bude jednať o softvérovú simuláciu, teda namie sto prenosu fyzických fotónov budú fotóny iba softvérovo simulované

Cieľom  tejto  misie  je  okrem  poukázania  na  rastúci  význam  kvantových  technológií,  predovšetkým zdôrazniť potrebu ochrany citlivých informácií (ako potenciálnych dát ktoré CanSat môže v budúcnosti prenášať)  v  kontexte  rastúcich  kybernetických  hrozieb.  Zatiaľ však  hlavný  telemetrický  prenos  zostáva

oddelený  od  QKD  simulácie,  ktorá  je  využitá  výlučne  na  testovaciu  výmenu  správ  medzi  sondou  a pozemnou stanicou.

Simulácia QKD taktiež vytvára praktický edukačný nástroj pre výučbu kvantovej kryptografie.

Súčasťou misie je aj využitie telemetrických dát na generovanie pseudonáhodných sekvencií, ktoré slúžia ako  vstup  pre  kryptografické  procesy.  Tento  prístup  demonštruje  potenciál  kombinácie  klasických systémov s kvantovou kryptografiou.

V dostupnej literatúre sa nám nepodarilo identifikovať obdobnú implementáciu simulácie QKD v rámci študentských projektov typu CanSat, čo podčiarkuje inovatívny a demonštračný charakter tejto práce.

## 6.3.3 Teoretický základ kvantovej distribúcie kľúčov

Protokol  BB84  pracuje  s  jednotlivými  fotónmi,  ktoré  sa  nachádzajú  v  štyroch  rôznych  polarizačných stavoch: horizontálny (H, 0°), vertikálny (V, 90°), diagonálny (D, 45°) a antidiagonálny (AD, 135°), ako je znázornené  na  obrázku.  Polarizácia  predstavuje  sme rovú  vlastnosť  fotónu,  teda  smer,  v  ktorom  je orientovaný.

<!-- image -->

Tieto smery sa zoskupujú do tzv. báz. Báza pozostáva vždy z dvoch navzájom kolmých smerov, ktoré slúžia ako základné orientácie pri meraní alebo popise polarizácie fotónu. Protokol BB84 využíva rektilineárnu bázu (H/V) a diagonálnu bázu (D/AD). V každej bá ze je jednotlivým polarizáciám priradená bitová hodnota 0 alebo 1 (vid tabuľku).

| Báza              | Polarizácia    | Označenie   |   Bitová hodnota |
|-------------------|----------------|-------------|------------------|
| Rektilineárna (+) | horizontálna   | H(0°)       |                0 |
| Rektilineárna (+) | vertikálna     | V(90°)      |                1 |
| Diagonálna (x)    | diagonálna     | D(45°)      |                0 |
| Diagonálna (x)    | antidiagonálna | AD(135°)    |                1 |

Vďaka kvantovým vlastnostiam fotónu je pri meraní polarizácie v správnej báze výsledok deterministický, zatiaľ čo pri meraní v nesprávnej báze je výsledok náhodný. Tento princíp je v našom projekte modelovaný softvérovo.

## 6.3.4 Aplikácia protokolu BB84 v CanSate

|    | Krok                        | Postup pri protokole BB84                                          | Zmenyvnašej implementácii:                                                                                                                                                |
|----|-----------------------------|--------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|  1 | Generovanie náhodných bitov | Nastrane odosielateľa sa generuje náhodná sekvencia bitov (0 a 1). | Vnašomprojektesú tieto hodnoty na strane odosielateľa (pozemná stanica) odvodené z dát gyroskopu počas letu, čím sa využíva reálny fyzikálny proces ako zdroj náhodnosti. |

The diagram shows a series of electrical components, including a battery, a resistor, and a capacitor. The components are connected in a series, meaning that they all have the same voltage across them. The battery is connected to the resistor, which is connected to the capacitor. The capacitor is connected to the resistor, which is connected to the other components.

<!-- image -->

|   2. | Náhodná voľba báz             | Ku každému bitu odosielateľ náhodne priradí bázu, buď rektilineárnu (H/V), alebo diagonálnu (D/AD), a v rámci nej príslušnú polarizáciu zodpovedajúcu hodnote bitu. Súčasne si aj prijímateľ generuje vlastnú sekvenciu náhodných báz nezávisle od odosielateľa.                                                                                                | Aj tento výber báz jevnašom projekte realizovaný na základe dát z telemetrie.                                                                                                                                          |
|------|-------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|    3 | Kódovanie informácie          | Vreálnom systéme sú bity reprezentované polarizáciou odosielaných fotónov.                                                                                                                                                                                                                                                                                      | Vnašomprojekteje tento proces modelovaný digitálne, pričom kombinácia bitu a bázy určuje konkrétny stav (H, V, Dalebo AD).                                                                                             |
|    4 | Prenos informácie             | Vreálnych systémoch sú fotóny odosielané medzi odosielateľom a prijímateľom cez optické vlákna alebo vo voľnom priestore. Ak počas tohto prenosu dôjde k pokusu o odpočúvanie, teda k meraniu fotónov treťou stranou, ich kvantový stav sa môžezmeniť. Táto zm ena sa následne prejaví zvýšenou chybovosťou, ktorú je možné detegovať v procese kontroly kľúča. | Vnašomprojektesú dáta prenášané medzi pozemnou stanicou (odosielateľom) a CanSatom (prijímateľom) prostredníctvom rádiovej komunikácie. Narozdiel od reálneho QKDsanepoužívajú fotóny, ale klasický komunikačný kanál. |
|    5 | Meranie na strane prijímateľa | Prijímateľ vykonáva meranie prijatých stavov v bázach, ktoré si náhodne zvolil v kroku 2. Ak je fotón meraný v správnej báze, výsledok zodpovedá pôvodne odoslanému bitu. Vprípade, že je meranie vykonané v nesprávnej báze, výsledok je náhodný a prijímateľmôže získať hodnotu 0 alebo 1 s rovnakou pravdepodobnosťou.                                       | Vnašomprojekteje náhodnosť tohto procesu modelovaná opäť pomocou telemetrie.                                                                                                                                           |
|    6 | Porovnanie použitých báz      | Po prenose si odosielateľ a prijímateľ vymenia informáciu o tom, aké bázy použili, pričom neodhalia samotné bitové hodnoty. Táto komunikáciamôže prebiehať aj verejne, pretože samotné hodnoty bitov zostávajú utajené a potenciálny odpočúvateľ z nej nedokáže získať informácie potrebné na rekonštrukciu kľúča.                                              | Vnašomprojekteje tento krok identický.                                                                                                                                                                                 |
|    7 | Vytvorenie spoločného kľúča   | Zachovajú sa iba tie bity, pri ktorých bola použitá rovnaká báza na oboch stranách. Ich hodnoty by sa mali u odosielateľa aj prijímateľa zhodovať. Tieto bity následne tvoria spoločný tajný kľúč.                                                                                                                                                              | Vnašomprojekteje tento krok identický                                                                                                                                                                                  |
|    8 | Detekcia odpočúvania          | Časť vytvoreného kľúča sa použije na kontrolu chybovosti, pričom si odosielateľ a prijímateľ túto časť kľúča navzájom verejne porovnajú a vyhodnotia mieru zhody bitov.                                                                                                                                                                                         | Vnašomprojektebudúmožné chyby spôsobené šumomvrádiovom prenose keďže naša simulácia odpočúvatela nezahŕňa.                                                                                                             |

The hardware diagram, schematic, or pinout is a detailed representation of the electrical components and their connections. Here is a detailed description of the components and their connections:

### Components and Connections

#### **Power Supply:**
- **DC Power:**
  - **DC Power:** This is the primary power source for the system. It is connected to the power supply via a 120V AC input.

#### **DC Power:**
- **DC Power:** This is the secondary power source for the system. It is connected to the DC power supply via a 120V AC input.

#### **DC Power:**
- **DC Power:** This is the secondary power source for the system. It is connected to the DC power supply via a 120V AC input.

#### **DC Power:**
- **DC Power:** This is the secondary power source for the system. It is connected to the DC power supply via a

<!-- image -->

|    |                                               | Možné nezhody a chyby môžubyť spôsobenébuďnadmernýmšumomv citlivom kvantovom kanáli, alebo prítomnosťou odpočúvateľa, ktorý svojim zásahom zmenil prenášané dáta. Ak je percento nezhôd vo vzorke kľúča príliš vysoké, je potrebné kľúč zahodiť a celý proces generovania zopakovať.   |                                                                                                                                                                                                                                                                                                                                                                                          |
|----|-----------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|  9 | Overenie funkčnosti pomocou šifrovanej správy | Vytvorený kľuč sa použije na šifrovanie správvbežnom (nekvantovom) kanáli.                                                                                                                                                                                                             | Naoverenie funkčnosti generovaného kľúča sa použije vopred definovaná správa, ktorá je zašifrovanápomocou vytvoreného kľúča na strane odosielateľa a následne odoslaná prijímateľovi. Prijímateľ správu dešifrujepomocou rovnakého kľúča a výsledok sa porovná s pôvodnou správou. Keďže obsah správy je vopred známy, je možné presne vyhodnotiť chybovosť a úspešnosť celej simulácie. |

Tabuľka  znázorňuje  priebeh  tohto  protokolu  na  ilustračnom  príklade.  Pre  zjednodušenie  v  nej  nie  je zahrnutý šum ani fáza porovnania časti výsledného kľúča.

<!-- image -->

| Odosielateľov Bit          | 0   | 1    | 1   | 0   | 0    | 0   | 1   | 0   | 1    | 1   |
|----------------------------|-----|------|-----|-----|------|-----|-----|-----|------|-----|
| Odosielateľova Báza        | +   | +    | x   | x   | +    | x   | +   | +   | +    | x   |
| Odosielateľova Polarizácia | H   | V    | AD  | D   | H    | D   | V   | H   | V    | AD  |
| Prijímateľova Báza         | +   | x    | x   | x   | x    | +   | +   | +   | x    | +   |
| Prijímateľovo Meranie      | H   | D/AD | AD  | D   | D/AD | H/V | V   | H   | D/AD | H/V |
| Kľuč                       | 0   |      | 1   | 0   |      |     | 1   | 0   |      | 1/0 |

| Rektilineárna báza                   | +   |
|--------------------------------------|-----|
| Diagonálna báza                      | x   |
| Zhoda je reprezentovaná sivou farbou |     |

## 6.4 Napájanie

Celý systém je napájaný jednou 18650 batériou s 20A a 8C discharge rate, čo by malo byť pre našu potrebu viac ako dostatočné, keďže zatiaľ počítame s približne 6A pri maximálnom výkone. V sonde sú dva oddelené meniče napätia. Jeden pre logic moduly (3.3V) a druhý pre motory a kameru (10V).

| Komponent                    | Napätie   | Prúd (peak)   | Výkon   | Účinnosť meniča   | Prúd z batérie (3,7 V)   |
|------------------------------|-----------|---------------|---------|-------------------|--------------------------|
| 2×DCmotory (max. záťaž)      | 10V       | 700mA         | 7,00W   | 85%(boost)        | 2,22 A                   |
| DJI O4Air Unit (Max TX)      | 10V       | 1160mA        | 11,6W   | 85 %(boost)       | 3,34 A                   |
| ESP32 (WiFi/BT aktívne)      | 3,3V      | 240mA         | 0,79W   | 90 %(buck)        | 0,24 A                   |
| LoRa modul (20 dBmTX)        | 3,3V      | 120mA         | 0,40W   | 90 %(buck)        | 0,12 A                   |
| GPSmodulNEO (aktívna anténa) | 3,3V      | 200mA         | 0,66W   | 90 %(buck)        | 0,20 A                   |

Commented [BM23]: mozno to je 21700 neviem ktora. znova jano

Commented [BM24R23]: @Jan Marcel Besson   Študent

| GPSmodulNEO (pasívna anténa)   | 3,3V   | 50mA   | 0,17W   | 90 %(buck)   | 0,05 A   |
|--------------------------------|--------|--------|---------|--------------|----------|
| Senzory (BME688, IMU)          | 3,3V   | 1,73mA | 0,006W  | 90 %(buck)   | 0,002 A  |
| CELKOM(aktívna anténa)         |        |        | 19,36W  |              | 6,12 A   |
| CELKOM (pasívna anténa)        |        |        | 18,86W  |              | 5,97 A   |

## 6.5 Komunikačný systém

Na  komunikáciu  medzi  satelitom  a  pozemnou  stanicou  využívame  na oboch  stranách  LoRa  modul. Softvérovo používame náš protokol + LoRa FEC. V každom pakete posielame dáta iba z jedného senzora, čo zaručuje menšie pakety, a tým aj menšiu šancu porušenia paketu a prípadného zlyhania FEC.

869.525 MHz | EIRP 24.15 dBm |RX 8.0 dBi | Terrain: SRTM profile: (48.6223, 18.3352) bearing 240deg. 2 km, 300 pts, ASL 177183 m (base 180 m ASL, normalised to AGL)

Distance(km)

The hardware diagram, schematic, or pinout is a diagram or schematic that shows the connections and relationships between various components and elements. In this case, the diagram is labeled "Full Discrete Profile - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

<!-- image -->

Distance(km)

Modelizácia signálovej  propagácie  pre terén letiska  Malé  Bielice -Prievidza.  Systém na modelizáciu je uPDM, vytvorený pod projektom PolySense.

## 7. Softvér

Na  zabezpečenie  pohybu  sondy  počas  sekundárnej  misie  je  potrebné,  aby  sme  videli,  kde  sa  satelit nachádza a aké prekážky má pred sebou. Preto potrebujeme z neho vysielať a na pozemnej stanici prijímať video záznam v reálnom čase. Na toto sme sa rozhodli použiť DJI Air unit O4. Táto integrovaná kamera a vysielač zabezpečí prenos videa až do 10km čo je na naše účely dostatočné. Špecifikácie Dji Air unit O4:

- -Protokol a technológia: Prenos prebieha digitálne (kódovanie H.265 ) v pásmach 2,4 / 5,1 / 5,8 GHz . Systém automaticky prepína frekvencie pre potlačenie rušenia.
- -Kvalita  a  latencia: Video  v  rozlíšení 1080p/60fps s  nízkou  latenciou  (~31  ms)  umožňuje  presné ovládanie roveru v reálnom čase.
- -Príjem  a  pozemná  stanica: Pilot  prijíma  obraz  cez  okuliare DJI  Goggles  N3 .  Dáta  sú  z  okuliarov zdieľané cez USB -C do aplikácie DJI Fly na sekundárny monitor pre zvyšok tímu.
- -Antény: Konfigurácia 2T4R na okuliaroch zabezpečuje stabilitu signálu aj pri pohybe CanSatu počas klesania.

Po  dopade  prejde  satelit  do  sekundárnej  misie,  ktorej  hlavnou  úlohou  je  zber  dát  zo  senzorov,  a  ich zasielanie  na  pozemnú  stanicu.  Zároveň  program  sleduje,  či  z  pozemnej  stanice  neprišli  príkazy  a  či nedosiahol dostatočnú výšku na použitie alebo odpojenie padáka. Využívame OS FreeRTOS a celý program bude napísaný v jazyku C/C++. V pozadí prebieha proces QKD ako proof-of-concept.

Palubný  softvér  KFDS26  zabezpečuje  sekundárnu  misiu  satelitu:  kontinuálny  zber  dát  zo  senzorov, zostavovanie  telemetrických  rámcov,  ich  prenos  na  pozemnú  stanicu,  spracovanie  prichádzajúcich príkazov a riadenie stavu misie vrátane logiky pre nasadenie padá ka. Firmware beží na mikrokontroléri ESP32S3 pod operačným systémom FreeRTOS a je napísaný v jazykoch C/C++ s využitím frameworku ESP-IDF a build systému CMake

## 7.1 Hardvérové periférie

Satelit využíva platformu PolySenseV1, vytvorenú skupinou PolySense. Táto verzia platformy je osadená nasledujúcimi  zariadeniami:  BME688,  ICM-42688-P,  NEO-M9N,  a  E22-900M22S. Všetky  štyri  periférne zariadenia zdieľajú jednu zbernicu SPI2 (HSPI) pracujúcu na frekvencii 1 MHz, pričom každé zariadenie má vyhradený vlastný výber čipu (CS) :

| Periférium        | Funkcia                                | CSpin   |
|-------------------|----------------------------------------|---------|
| BME688            | Teplota, tlak, vlhkosť, plyny          | GPIO9   |
| ICM-42688-P       | 6-osový IMU (akcelerometer + gyroskop) | GPIO10  |
| u-bloxNEO-M9N     | GNSS/GPSpoloha                         | GPIO15  |
| Ebyte E22-900M22S | LoRa transceiver868MHz                 | GPIO14  |

## 7.2 Štruktúra úloh FreeRTOS

Firmware je rozdelený do piatich súbežných FreeRTOS úloh s pevne stanovenými veľkosťami zásobníka a prioritami :

Commented [BM25]: janova sekcia, netusim

Commented [BM26R25]: @Jan Marcel Besson   Študent

Commented [JŠ27R25]: moze byt cele vybavene

Commented [BM28]: podla mna vhodny kratky opis ale ak to uz ma byt detailne tak to mozem urobit

Commented [BM29]: @Alexandra Butašová -Študent

| Úloha        | Funkcia                                                                      | Zásobník   |   Priorita |
|--------------|------------------------------------------------------------------------------|------------|------------|
| sensor_task  | Číta BME688(1Hz) a ICM -42688-P (10 Hz); zostavujeENVaIMU telemetrické rámce | 4096B      |          5 |
| gnss_task    | ČítapolohuzNEO -M9N frekvenciou 1 Hz; zostavuje GPStelemetrické rámce        | 4096B      |          5 |
| lora_tx_task | Vyberá rámce z TX fronty a vysiela ich cez E22 LoRa                          | 4096B      |          6 |
| lora_rx_task | Nepretržite počúva na príkazy z pozemnej stanice;spracovávaCMD rámce         | 4096B      |          7 |
| status_task  | Pravidelne (1 Hz) vysiela STATUSrámce so stavom misie,RSSIaSNR               | 3072B      |          3 |

## 7.3 Komunikačný protokol PolySense

Všetky  downlinkové  aj  uplinkové  správy  používajú  binárny  protokol  PolySense  (verzia  0x01)  .  Štruktúra každého rámca:

- 2 bajty synchronizačného slova (0x55 0xAA)
- 5bajtová hlavička: verzia protokolu, typ správy, ID uzla, dĺžka payloadu

Payload premennej dĺžky (zabalené C štruktúry)

2 bajty kontrolného súčtu CRC16 -CCITT (polynóm 0x1021, init 0xFFFF)

Definované typy downlinkových správ :

| ID   | Typ       | Obsah                                                                                         |
|------|-----------|-----------------------------------------------------------------------------------------------|
| 0x01 | ENV_TELEM | Teplota (×100), tlak (Pa), vlhkosť (×100), odpor plynu (Ω)                                    |
| 0x02 | IMU_TELEM | Zrýchlenie XYZ (m/s²×1000), uhlová rýchlosť XYZ (°/s×1000), kvaternión                        |
| 0x03 | GPS_TELEM | Zemepisná šírka/dĺžka (×1e7), nadmorská výška (×100), rýchlosť,HDOP,počet satelitov, typ fixu |
| 0x04 | STATUS    | Uptime, napätie batérie, stav misie, RSSI, SNR                                                |
| 0x05 | BEACON    | Kompaktný záchranný maják s poslednou GPSpolohou                                              |

## 7.4 Konfigurácia rádiového modulu LoRa

Modul E22-900M22S je nakonfigurovaný nasledovne :

Frekvencia: 868 MHz (európske ISM pásmo) Spreading Factor: SF7 Šírka pásma: 125 kHz

Kódovací pomer: 4/5 Vysielací výkon: 22 dBm (maximum modulu) Synchronizačné slovo: 0x12 (privátna sieť)

## 7.5 Spracovanie príkazov a stavový automat misie

Úloha  lora\_rx\_task  podáva  prijaté  bajty  do  prúdového  parsera  ps\_parser\_t.  Po  úspešnom  dekódovaní rámca  sú  správy  typu  PS\_MSG\_CMD  odovzdané  funkcii  handle\_command()  .  Príkazy  sú  rozdelené  do štyroch tried :

RECOVERY -aktivácia/deaktivácia majáka, režim záchrannej misie MOTOR -riadenie aktuátorov (rezerva pre budúce použitie) MISSION -prechody  stavov  (IDLE  →  PRELAUNCH → ASCENT → DESCENT → LANDED →  RECOVERY) a nastavenie frekvencie telemetrie CONFIG -konfigurácia systému

Každý príkaz dostane späť rámec CMD\_ACK  (0x11) so stavovým kódom (ACCEPTED, DONE, REJECTED\_INVALID a pod.) a ID príkazu na párovanie . Stav misie je chránený mutexom; prechody sú zaznamenávané  cez  ESP\_LOGI.  Logika  pre  nasadenie  a  odpojenie  padáka  je  implementovaná  ako podmienka  vyhodnocovaná  v  stavovom  automate  počas  prechodu  ASCENT  →  DESCENT,  na  základe nadmorskej výšky poskytnutej úlohou gnss\_task .

## 7.6 Vývojové prostredie a build systém

Projekt cieli na ESP-IDF v5.x s CMake ako build systémom. Skript setup\_dev\_env.sh automatizuje prípravu vývojového prostredia a súbor sdkconfig.defaults fixuje kľúčové nastavenia IDF (frekvencia tikania FreeRTOS, SPI ovládač, úroveň logovania) pre reprodukovateľné buildy. Zdrojový kód je dostupný na github.com/onyx-the-one/kfds v adresári KFDS26/main/node/.

1. Návratový systém Návratový systém je navrhnutý tak, aby zabezpečil stabilitu počas letu a klesanie v rámci limitov stanovených pravidlami ESA CanSat (8 m/s až 11 m/s). Rozhodli sme sa použiť  padák  pretože  je  to  najjednoduchšie  a  najspoľahlivejšie  riešenie.  Používame  polo -g uľový padák, pretože je najefektívnejší, a teda môže byť menší ako iné typy padákov.

2.

#1: Vipolet Plocher
#2: Vipolet Plocher
#3: Vipolet Polemeru
#4: Vipolet Polemeru
#5: Vipolet Plocher
#6: Vipolet Plocher
#7: Vipolet Plocher
#8: Vipolet Plocher
#9: Vipolet Plocher
#10: Vipolet Plocher
#11: Vipolet Plocher
#12: Vipolet Plocher
#13: Vipolet Plocher
#14: Vipolet Plocher
#15: Vipolet Plocher
#16: Vipolet Plocher
#17: Vipolet Plocher
#18: V

<!-- image -->

3.

4. Vytvorili sme preto papierový testovací padák s priemerom 20 cm. No po testoch sme zistili že tento priemer nebude dostatočný. Po zhodení padáku so závažím 350g z 7 metrov sme vypočítali konečnú rýchlosť na základe záberov z kamery -Konečná rýchlosť: 11.21 m/s Čo prekračuje limit
5. Padák sme preto zväčšili na 26cm -Konečná rýchlosť: 9m/s
6. Po tom čo sme mali veľkosť otestovanú sme padák vyrobili z polyamidovej tkaniny RipStop. Tento materiál je ľahký, pevný a vďaka jeho štruktúre odolný voči šíreniu roztrhnutia, a zaberá málo miesta čo ho robí ideálnym na použitie v návratovom systéme.
7. Na zabezpečenie polkruhového tvaru je padák zošitý z 6 častí. K sonde je priviazaný 6 šnúrami, ktorých odolnosť sme otestovali zavesením 10kg závažia na ich koniec.

8.

<!-- image -->

9.

10. Týmto testom sme overili, že padák ma dostatočnú rezervu a je dosť odolný.

<!-- image -->

## 13. Pozemná stanica

Pozemná stanica sa bude skladať z troch častí:

- -LoRa modul -zachytávanie signálu a kontrola/opravovanie defektovaných paketov pomocou FEC a CRC
- -Raspberry pico 2 -číta dáta z LoRa modulu a posúva ich ďalej do počíta s našim softwareom a vice versa,
- -zariadenie s našou aplikáciou spracúva dáta, kreslí grafy a ovláda CanSat po transformácii na rover. V tomto prípade to je osobný laptop.

## 9.1 Pozemná stanica -softvér

Všetk o overovanie integrity dát rieši LoRa modul, ktory bude v auton ómnom nastavení. Softvér pozemnej stanice sa bude teda skladať z 2 častí: 1.  Je  napísaná  v  programovacom  jazyku  C/C++.  Pri  inicializácii  sa  LoRa  modul  nastaví  pod ľ a  vopred definovaných  parametrov, a  funguje  iba  ako  komunikačný  most  medzi počítačom  a  LoRa  modulom. 2. Počítačová aplikácia je napísaná v programovacom jazyku Python. Preberá dáta z mikrokontroléra cez UART-to-USB  most  a  v  základnej  konfigurácii vykresluje  pomocou  matplotlib  pythonovej  knižnice.

Softvér  pozemnej  stanice  ktorý  je  napísaný  v  Pythone  obsahuje  implementáciu  rovnakého  protokolu PolySense ako v samotnom CanSate, vrátane redundantného výpočtu CRC16 -CCITT, kódovacích funkcií pre každý typ správy a prúdového parsera FrameParser. Parser postupne spracováva prijaté bajty, overuje synchronizáciu  a  integritu  CRC  (ako  sekundárnu  kontrolu),  a  vracia  dekódované  objekty  (EnvTelem, ImuTelem,  GpsTelem, Status, Beacon, CmdAck, a iné). Hlavný  program je  v  čase  CDR  ešte  vo  vývoji , disponujeme tiež testovaciou konzolou ktorá slúži na overenie komunikácie počas integrácie.

## 14. Promo

Ako a akými kanálmi bude projekt prezentovaný počas práce a po jej ukončení? Koho chcete prezentáciou projektu osloviť?

Náš  projekt  prezentujeme  viacerými  spôsobmi,  aby  sme  oslovili  čo  najširšie  publikum.  Počas  Dňa otvorených  dverí  na  našej  škole  sme  ho  predstavili  a  mali  možnosť  priblížiť  študentom,  učiteľom  a návštevníkom, na čom pracujeme. Táto prezentácia nám pomohla nielen zvýšiť povedomie o projekte, ale aj získať cennú spätnú väzbu.

Minulý rok sa nám najviac osvedčil na propagáciu instagramový účet, kde sme zverejňovali fotografie a videá  dokumentujúce  náš  pokrok,  naše  úspechy,  ale  aj  neúspechy  a získali  sme  viac  ako  13  tisíc zahliadnutí. Preto plánujeme touto formou pokračovať. Cieľom je osloviť nielen školskú komunitu, ale aj širšiu verejnosť a potenciálnych sponzorov. Plánujeme naďalej pridávať obsah, aby sme udržali záujem sledujúcich Náš instagram účet: cansatkfds

Okrem toho chceme vytvárať dlhšie videá na YouTube, ktoré budú technickejšie a podrobnejšie, s cieľom osloviť skôr technicky zameraných ľudí.