// Compression rollback backup bounds regression.
//
// A streaming recovery restores the raw in-flight batch from
// fifo_buffer_backup. Production batches can exceed 4 KiB, so the restore
// helper must size its temporary from actual occupancy and preserve the entire
// byte sequence ahead of newer queued data.

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include <algorithm>
#include <cstdio>
#include <vector>

int cl_arq_controller::test_restore_backup_bounds()
{
	const int backup_len = 44000;
	const int newer_len = 2000;
	const int fifo_size = 128000;

	max_data_length = 194;
	max_header_length = 6;
	sack_v2_enabled = true;
	header_carries_d5 = false;

	if(fifo_buffer_backup.set_size(fifo_size) != SUCCESSFUL
	   || fifo_buffer_tx.set_size(fifo_size) != SUCCESSFUL)
	{
		printf("[TEST-RESTORE-BOUNDS] FAIL: FIFO allocation\n");
		return 1;
	}
	fifo_buffer_backup.flush();
	fifo_buffer_tx.flush();

	std::vector<char> backup((size_t)backup_len);
	std::vector<char> newer((size_t)newer_len);
	for(int i=0;i<backup_len;i++) backup[(size_t)i]=(char)((i*37+11)&0xff);
	for(int i=0;i<newer_len;i++) newer[(size_t)i]=(char)((i*19+7)&0xff);

	int backup_pushed=fifo_buffer_backup.push(backup.data(),backup_len);
	int newer_pushed=fifo_buffer_tx.push(newer.data(),newer_len);
	printf("[TEST-RESTORE-BOUNDS] staged backup=%d newer=%d\n",
		backup_pushed,newer_pushed);
	fflush(stdout);
	if(backup_pushed!=backup_len || newer_pushed!=newer_len) return 1;

	restore_backup_buffer_data();

	std::vector<char> got((size_t)backup_len+newer_len);
	int popped=fifo_buffer_tx.pop(got.data(),(int)got.size());
	bool backup_empty=(fifo_buffer_backup.get_free_size()==fifo_buffer_backup.get_size());
	bool length_ok=(popped==(int)got.size());
	bool bytes_ok=length_ok
		&& std::equal(backup.begin(),backup.end(),got.begin())
		&& std::equal(newer.begin(),newer.end(),got.begin()+backup_len);

	printf("[TEST-RESTORE-BOUNDS] restored=%d backup_empty=%d bytes_ok=%d\n",
		popped,backup_empty?1:0,bytes_ok?1:0);
	printf("[TEST-RESTORE-BOUNDS] %s\n",
		(backup_empty&&bytes_ok)?"ALL PASS":"FAILURES");
	fflush(stdout);
	return (backup_empty&&bytes_ok)?0:1;
}
