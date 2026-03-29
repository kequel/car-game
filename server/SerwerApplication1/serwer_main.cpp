/****************************************************
 * VCT - Serwer (architektura klient-serwer)
 * Kompilacja: dolacz net.cpp, vector3D.cpp, quaternion.cpp
 * Biblioteki: WSOCK32.LIB, MPR.LIB
 ****************************************************/

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <map>
#include <vector>

#include "net.h"
#include "objects.h"   // potrzebne dla ObjectState, Vector3

using namespace std;

// -------------------------------------------------------
// Typy ramek (musza byc zgodne z klientem!)
// -------------------------------------------------------
#define FRAME_STATE     1
#define FRAME_COLLISION 2
#define FRAME_REGISTER  3   // klient rejestruje sie u serwera
#define FRAME_INACTIVE  4   // serwer informuje o nieaktywnym kliencie

// Timeout klienta: jesli przez CLIENT_TIMEOUT_MS ms nie przyszla zadna ramka
// od klienta, uznajemy go za nieaktywnego
#define CLIENT_TIMEOUT_MS 5000

// Porty:
//   Serwer ODBIERA na porcie SERVER_RECV_PORT
//   Serwer WYSYLA  na porcie SERVER_SEND_PORT  (klient odbiera na tym porcie)
#define SERVER_RECV_PORT  htons(1001)
#define SERVER_SEND_PORT  htons(1002)

// Promien sfery opisujacej pojazd (uproszczenie - taka sama dla wszystkich)
#define BOUNDING_RADIUS 4.5f

// -------------------------------------------------------
// Ramka sieciowa (identyczna jak w kliencie)
// -------------------------------------------------------
struct Frame
{
    int  iID;
    int  type;
    ObjectState state;
    long sending_time;
    int  iID_receiver;   // 0 = broadcast do wszystkich klientow
};

// -------------------------------------------------------
// Informacje o zarejestowanym kliencie
// -------------------------------------------------------
struct ClientInfo
{
    unsigned long ip;          // adres IP klienta (network byte order)
    ObjectState   last_state;  // ostatni znany stan
    long          last_seen;   // clock() w chwili ostatniej ramki
    bool          active;
};

// -------------------------------------------------------
// Globalne dane serwera
// -------------------------------------------------------
map<int, ClientInfo> clients;   // klucz: iID klienta
unicast_net* srv_recv = NULL;   // odbior od klientow
unicast_net* srv_send = NULL;   // wysylanie do klientow
CRITICAL_SECTION cs;

FILE* f = fopen("server_log.txt", "w");

// -------------------------------------------------------
// Wysylanie ramki do konkretnego klienta (po IP)
// -------------------------------------------------------
void SendToClient(unsigned long ip, Frame& fr)
{
    srv_send->send((char*)&fr, ip, sizeof(Frame));
}

// -------------------------------------------------------
// Wysylanie ramki do wszystkich aktywnych klientow
// (opcjonalnie pomijamy klienta o iID == skip_id)
// -------------------------------------------------------
void Broadcast(Frame& fr, int skip_id = -1)
{
    for (auto& kv : clients)
    {
        if (!kv.second.active) continue;
        if (kv.first == skip_id) continue;
        SendToClient(kv.second.ip, fr);
    }
}

// -------------------------------------------------------
// Sprawdzenie kolizji miedzy wszystkimi parami klientow
// Kolizja jest wykrywana przez serwer i komunikowana obu stronom
// -------------------------------------------------------
void CheckCollisions()
{
    EnterCriticalSection(&cs);

    vector<int> ids;
    for (auto& kv : clients)
        if (kv.second.active) ids.push_back(kv.first);

    for (size_t i = 0; i < ids.size(); i++)
    {
        for (size_t j = i + 1; j < ids.size(); j++)
        {
            int idA = ids[i], idB = ids[j];
            ClientInfo& A = clients[idA];
            ClientInfo& B = clients[idB];

            Vector3 diff = A.last_state.vPos - B.last_state.vPos;
            float dist = diff.length();

            if (dist < 2.0f * BOUNDING_RADIUS)
            {
                // Wyslij FRAME_COLLISION do klienta A (mowi mu, ze zderza sie z B)
                Frame fA;
                fA.iID = idB;        // sprawca (zrodlo kolizji)
                fA.type = FRAME_COLLISION;
                fA.state = B.last_state;
                fA.iID_receiver = idA;
                fA.sending_time = clock();
                SendToClient(A.ip, fA);

                // Wyslij FRAME_COLLISION do klienta B (mowi mu, ze zderza sie z A)
                Frame fB;
                fB.iID = idA;
                fB.type = FRAME_COLLISION;
                fB.state = A.last_state;
                fB.iID_receiver = idB;
                fB.sending_time = clock();
                SendToClient(B.ip, fB);

                fprintf(f, "[KOLIZJA] klient %d <-> klient %d, dist=%.2f\n",
                    idA, idB, dist);
                fflush(f);
            }
        }
    }

    LeaveCriticalSection(&cs);
}

// -------------------------------------------------------
// Sprawdzenie timeoutow klientow i informowanie pozostalych
// -------------------------------------------------------
void CheckTimeouts()
{
    long now = clock();
    EnterCriticalSection(&cs);

    for (auto& kv : clients)
    {
        if (!kv.second.active) continue;
        long elapsed_ms = (long)(((float)(now - kv.second.last_seen) / CLOCKS_PER_SEC) * 1000.0f);
        if (elapsed_ms > CLIENT_TIMEOUT_MS)
        {
            kv.second.active = false;
            fprintf(f, "[TIMEOUT] klient %d nieaktywny\n", kv.first);
            fflush(f);

            // Powiadom pozostalych
            Frame fi;
            fi.iID = kv.first;
            fi.type = FRAME_INACTIVE;
            fi.iID_receiver = 0;
            fi.sending_time = clock();
            Broadcast(fi);
        }
    }

    LeaveCriticalSection(&cs);
}

// -------------------------------------------------------
// Watek odbioru ramek od klientow
// -------------------------------------------------------
DWORD WINAPI ReceiveThread(void* /*ptr*/)
{
    Frame frame;
    unsigned long senderIP = 0;

    while (1)
    {
        int len = srv_recv->reciv((char*)&frame, &senderIP, sizeof(Frame));
        if (len <= 0) continue;

        EnterCriticalSection(&cs);

        // Rejestracja / aktualizacja klienta
        if (clients.find(frame.iID) == clients.end())
        {
            ClientInfo ci;
            ci.ip = senderIP;
            ci.last_state = frame.state;
            ci.last_seen = clock();
            ci.active = true;
            clients[frame.iID] = ci;
            fprintf(f, "[REJ] nowy klient iID=%d  IP=%lu\n", frame.iID, senderIP);
            fflush(f);
        }
        else
        {
            clients[frame.iID].ip = senderIP;
            clients[frame.iID].last_state = frame.state;
            clients[frame.iID].last_seen = clock();
            clients[frame.iID].active = true;
        }

        LeaveCriticalSection(&cs);

        // Retransmisja stanu do pozostalych klientow (nie odsylamy nadawcy)
        if (frame.type == FRAME_STATE)
        {
            Broadcast(frame, frame.iID);
        }
    }
    return 1;
}

// -------------------------------------------------------
// main
// -------------------------------------------------------
int main()
{
    InitializeCriticalSection(&cs);

    printf("=== VCT Serwer ===\n");
    fprintf(f, "=== VCT Serwer start ===\n");

    // Serwer odbiera na porcie 1001 (klienci nadaja na 1001)
    srv_recv = new unicast_net(SERVER_RECV_PORT);
    // Serwer wysyla z portu 1002 (klienci odbieraja na 1002)
    srv_send = new unicast_net(SERVER_SEND_PORT);

    printf("Porty otwarte. Oczekiwanie na klientow...\n");

    // Uruchom watek odbioru
    DWORD tid;
    HANDLE hThread = CreateThread(NULL, 0, ReceiveThread, NULL, 0, &tid);
    SetThreadPriority(hThread, THREAD_PRIORITY_HIGHEST);

    long last_collision_check = clock();
    long last_timeout_check = clock();

    while (1)
    {
        long now = clock();

        // Co ~100ms sprawdzaj kolizje
        if ((now - last_collision_check) * 1000 / CLOCKS_PER_SEC >= 100)
        {
            CheckCollisions();
            last_collision_check = now;
        }

        // Co ~1s sprawdzaj timeouty
        if ((now - last_timeout_check) * 1000 / CLOCKS_PER_SEC >= 1000)
        {
            CheckTimeouts();
            last_timeout_check = now;

            // Wypisz aktywnych klientow
            EnterCriticalSection(&cs);
            printf("Aktywni klienci: ");
            for (auto& kv : clients)
                if (kv.second.active) printf("%d ", kv.first);
            printf("\n");
            LeaveCriticalSection(&cs);
        }

        Sleep(10);
    }

    return 0;
}