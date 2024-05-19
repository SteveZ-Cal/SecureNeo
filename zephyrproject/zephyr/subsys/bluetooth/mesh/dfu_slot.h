/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/mesh.h>

/** @brief Slot iteration callback.
 *
 *  @param slot      A valid DFU image slot.
 *  @param user_data User data passed to @ref bt_mesh_dfu_slot_foreach.
 *
 *  @return Iteration action determining next step.
 */
typedef enum bt_mesh_dfu_iter (*bt_mesh_dfu_slot_cb_t)(
	const struct bt_mesh_dfu_slot *slot, void *user_data);

/** @brief Register a new DFU image slot for a distributable image.
 *
 *  A DFU image slot represents a single distributable DFU image with all its
 *  metadata.
 *
 *  @note The slot is allocated as invalid. Call
 *        @ref bt_mesh_dfu_slot_valid_set to make it valid.
 *
 *  @param size         Size of the image in bytes.
 *  @param fwid         Firmware ID.
 *  @param fwid_len     Length of the firmware ID, at most @c
 *                      CONFIG_BT_MESH_DFU_FWID_MAXLEN.
 *  @param metadata     Metadata or NULL.
 *  @param metadata_len Length of the metadata, at most @c
 *                      CONFIG_BT_MESH_DFU_METADATA_MAXLEN.
 *  @param uri          Image URI or NULL.
 *  @param uri_len      Length of the image URI, at most @c
 *                      CONFIG_BT_MESH_DFU_URI_MAXLEN.
 *
 *  @return A pointer to the allocated slot, or NULL if allocation failed.
 */
const struct bt_mesh_dfu_slot *
bt_mesh_dfu_slot_add(size_t size, const uint8_t *fwid, size_t fwid_len,
		     const uint8_t *metadata, size_t metadata_len,
		     const char *uri, size_t uri_len);

/** @brief Set whether the given slot is valid.
 *
 *  @param slot  Allocated DFU image slot.
 *  @param valid New valid state of the slot.
 *
 *  @return 0 on success, or (negative) error code on failure.
 */
int bt_mesh_dfu_slot_valid_set(const struct bt_mesh_dfu_slot *slot, bool valid);

/** @brief Check whether a slot is valid.
 *
 *  @param slot Slot to check.
 *
 *  @return true if the slot is valid, false otherwise.
 */
bool bt_mesh_dfu_slot_is_valid(const struct bt_mesh_dfu_slot *slot);

/** @brief Delete an allocated DFU image slot.
 *
 *  @param slot Slot to delete. Must be a valid pointer acquired from this
 *              module.
 *
 *  @return 0 on success, or (negative) error code on failure.
 */
int bt_mesh_dfu_slot_del(const struct bt_mesh_dfu_slot *slot);

/** @brief Delete all DFU image slots.
 *
 *  @return 0 on success, or (negative) error code on failure.
 */
int bt_mesh_dfu_slot_del_all(void);

/** @brief Get the DFU image slot at the given index.
 *
 *  @param idx DFU image slot index.
 *
 *  @return The DFU image slot at the given index, or NULL if no slot exists with the
 *          given index.
 */
const struct bt_mesh_dfu_slot *bt_mesh_dfu_slot_at(uint16_t idx);

/** @brief Get the DFU image slot for the image with the given firmware ID.
 *
 *  @param fwid     Firmware ID.
 *  @param fwid_len Firmware ID length.
 *  @param slot     Slot pointer to fill.
 *
 *  @return Slot index on success, or negative error code on failure.
 */
int bt_mesh_dfu_slot_get(const uint8_t *fwid, size_t fwid_len,
			 const struct bt_mesh_dfu_slot **slot);

/** @brief Get the DFU image slot index of the given slot.
 *
 *  @param slot Slot to find.
 *
 *  @return Slot index on success, or negative error code on failure.
 */
int bt_mesh_dfu_slot_idx_get(const struct bt_mesh_dfu_slot *slot);

/** @brief Iterate through all DFU image slots.
 *
 *  Calls the callback for every DFU image slot or until the callback returns
 *  something other than @ref BT_MESH_DFU_ITER_CONTINUE.
 *
 *  @param cb        Callback to call for each slot, or NULL to just count the
 *                   number of slots.
 *  @param user_data User data to pass to the callback.
 *
 *  @return The number of slots iterated over.
 */
size_t bt_mesh_dfu_slot_foreach(bt_mesh_dfu_slot_cb_t cb, void *user_data);
