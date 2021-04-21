#include "Keys.h"
#include <Windows.h>

namespace UE_Utilities {

	HWND hWnd = NULL;
	struct HandleData
	{
		DWORD pid;
		HWND hWnd;
	};
	HWND
		FindMainWindow(DWORD dwPID);
	BOOL CALLBACK
		EnumWindowsCallback(HWND hWnd, LPARAM lParam);

	HWND
		FindMainWindow(DWORD dwPID)
	{
		HandleData handleData{ 0 };
		handleData.pid = dwPID;
		EnumWindows(EnumWindowsCallback, (LPARAM)& handleData);
		return handleData.hWnd;
	}

	BOOL CALLBACK EnumWindowsCallback(HWND hWnd, LPARAM lParam)
	{
		HandleData& data = *(HandleData*)lParam;
		DWORD pid = 0;
		GetWindowThreadProcessId(hWnd, &pid);
		if (pid == data.pid && GetWindow(hWnd, GW_OWNER) == HWND(0) &&
			IsWindowVisible(hWnd)) {
			data.hWnd = hWnd;
			return FALSE;
		}

		return TRUE;
	}

	void getHWND(void) {
		hWnd = FindMainWindow(GetCurrentProcessId());
	}

	KeyManager keyManager;

	Key::Key(int key, void (*onState)(void), void (*offUnHoldState)(void), void (*onHoldState)(void), void (*offState)(void)) {
		this->key = key;
		this->onState = onState;    // key went from off to on state
		this->offState = offState;  // key is on a constant off state

		this->onHoldState = onHoldState;        // key is being held at constant on state
		this->offUnHoldState = offUnHoldState;  // key went from on state to off state
	}

	bool Key::checkState(bool checkhwnd) {
          bool isFocused = true;
          if (checkhwnd)
			  isFocused = (hWnd == GetForegroundWindow());

		if (isFocused && GetAsyncKeyState(key) && !flag) {
			flag = true;
			if (onState)
				onState();
			return true;
		}
		else if (GetAsyncKeyState(key)) {
			if (onHoldState)
				onHoldState();
			return true;
		}
		else if (GetAsyncKeyState(key) == 0 && flag) {
			flag = false;
			if (offUnHoldState)
				offUnHoldState();
		}
		else if (GetAsyncKeyState(key) == 0) {
			if (offState)
				offState();
			return false;
		}
		return false;
	}
}

namespace UE_Utilities {
	void KeyManager::addKey(Key key) {
		keys.push_back(key);
	}

	void KeyManager::addKey(int key, void (*onState)(void), void (*offUnHoldState)(void), void (*onHoldState)(void), void (*offState)(void)) {
		keys.push_back(Key(key, onState, offUnHoldState, onHoldState, offState));
	}

	void KeyManager::checkKeyStates(bool checkhwnd) {
		pressedKeys.clear();
		for (std::vector<Key>::iterator i = keys.begin(); i != keys.end(); i++) {
			if (i->checkState(checkhwnd))
				pressedKeys.push_back(*i);
		}
	}

	std::vector<Key> KeyManager::getPressedKeys(void) {
		return pressedKeys;
	}
}